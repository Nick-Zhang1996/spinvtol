# binary protocol version of groundstation.py
# data processor/logger for monocopter avionics with vicon
import serial
import curses
from curses.textpad import Textbox, rectangle
import datetime,time,platform
import numpy as np
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import warnings
from math import pi,sin,cos
import matplotlib.pyplot as plt
from time import time,sleep
from vicon import Vicon
import threading
from threading import Lock
from struct import pack,unpack
import os.path
import pygame
from dummyController import expControl,cvt2pwm

# -------  Settings ------------
formatISfullsensor = False
enablePlot = False
enableVicon = False
enableJoystick = False
enableController = False
logFolder = "./log/"
logPrefix = "richrun"
logSuffix = ".txt"

# from bottom, how far up do we display rolling data
rolling_offset = 10
screenHandle = None

# ---- threading
# vicon_state: (x,y,z,rx,ry,rz)
vicon_state = None
lock_vicon_state = Lock()
thread_vicon = None
thread_joystick = None

# avionics_state: voltage,flapPWM,throttlePWM,isTelemCtrl
avionics_state = None
lock_avionics_state = Lock()

# joystick locks etc
js_flap = None
js_throttle = None
lock_js = Lock()

lock_logfile = Lock()
quit = False
isLogging = False
logFilename = None
logBuffer = []

# controller specific
vicon_list = []
remote_flapPWM = None
remote_throttlePWM = None
flag_new_remote = False


def displayText(text):
    global ymax,xmax,rolling_offset, screenHandle
    screen = screenHandle
    #display a human readable text slip
    screen.scroll()
    screen.addstr(ymax-rolling_offset,0,text)
    return

def displayLineno(n,text):
    global screenHandle
    screen = screenHandle
    # display text at line n (counting from bottom up, bottom line is 1)
    assert n>0
    screen.move(ymax-n,0)
    screen.clrtoeol()
    screen.addstr(ymax-n,0,text)

debug_joy_update_interval_ms = 0.0
debug_joy_ts = 0.0
def joystickUpdateDaemon(js,avionics):
    global js_throttle,js_flap,lock_js,quit,enableJoystick
    global debug_joy_update_interval_ms,debug_joy_ts
    fmap = lambda x,a,b,c,d: c+(x-a)/(b-a)*(d-c)
    while (not quit and enableJoystick):
        pygame.event.pump()
        throttle = fmap(js.get_axis(5),-1.0,1.0,1098,1939)
        flap = fmap(js.get_axis(1),-1.0,1.0,1099,1939)
        lock_js.acquire()
        js_throttle = throttle
        js_flap = flap
        lock_js.release()
        #avionics.write(("ctrl %d %d\r\n"%(throttle,flap)).encode())
        
        outdata = bytearray(5)
        # message type 1:control update 2:ping request
        outdata[0] = 1
        # flap pwm
        outdata[1:3] = pack('H',flap)
        # throttle pwm
        outdata[3:] = pack('H',throttle)
        outcount = avionics.write(outdata)

        debug_joy_update_interval_ms = (time()-debug_joy_ts)*1000
        debug_joy_ts = time()

        sleep(0.03)

    js_throttle = None
    js_flap = None
        


def viconUpdateDaemon(vi,avionics):
    global quit,lock_vicon_state,vicon_state,avionics_state,lock_avionics_state
    global isLogging,logFilename,logBuffer,lock_logfile
    global vicon_list,enableController,remote_throttlePWM,remote_flapPWM,flag_new_remote
    while (not quit):
        local_vicon_state = vi.getViconUpdate()
        lock_vicon_state.acquire()
        vicon_state = local_vicon_state
        lock_vicon_state.release()

        lock_avionics_state.acquire()
        local_avionics_state = avionics_state
        lock_avionics_state.release()
        if (local_vicon_state is None):
            # should not happen
            continue
        voltage,flapPWM,throttlePWM,isTelemCtrl = local_avionics_state

        if (isLogging):
            x,y,z,rx,ry,rz = local_vicon_state

            dataFrame = str(time())+","+str(x)+","+str(y)+","+str(z)+","+str(rx)+","+str(ry)+","+str(rz)+","+str(throttlePWM)+","+str(flapPWM)+","+str(voltage)+","+ str(1 if isTelemCtrl else 0) + "\n"
            logBuffer.append(dataFrame)
            if len(logBuffer)>1000:
                lock_logfile.acquire()
                with open(logFilename, 'a') as filehandle:
                    for entry in logBuffer:
                            filehandle.write('%s' % entry)
                logBuffer = []
                lock_logfile.release()

        if (enableController):
            if (len(vicon_list)<5):
                vicon_list.append(local_vicon_state)
            else:
                #thrust_N,flap_rad = expControl(vicon_list)
                #remote_flapPWM,remote_throttlePWM = cvt2pwm(thrust_N,flap_rad,voltage)
                remote_flapPWM,remote_throttlePWM = (1300,1500)
                vicon_list = []
                outdata = bytearray(5)
                # message type 1:control update 2:ping request
                outdata[0] = 1
                # flap pwm
                outdata[1:3] = pack('H',remote_flapPWM)
                # throttle pwm
                outdata[3:] = pack('H',remote_throttlePWM)
                outcount = avionics.write(outdata)
                flag_new_remote = True

    return


def main(screen, avionics):
    global screenHandle,formatISfullsensor,quit
    global enableVicon,enableJoystick,enablePlot
    global thread_joystick,thread_vicon
    global lock_logfile,lock_avionics_state,lock_vicon_state,lock_js
    global vicon_state,avionics_state,js_flap,js_throttle
    global isLogging,logFilename,logBuffer
    global debug_joy_update_interval_ms
    global enableController,remote_throttlePWM,remote_flapPWM,flag_new_remote,vicon_list



    screenHandle = screen
    curses.curs_set(0)                      # Set cursor visibility where 0 = invisible,
    screen.nodelay(True)                    # Set getch() to be non-blocking - important!
    curses.noecho()
    curses.use_default_colors()             # Lets your terminal client set the fonts and colors
    global ymax,xmax
    ymax, xmax = screen.getmaxyx()          # Get the screen dimensions
    screen.setscrreg(0,ymax-rolling_offset)
    screen.scrollok(True)

    command = ""
    # ---------- display format ------------
    # --- human readable text ----- (scrollable,non-erasing)
    # line 5: joystick output, throttle, flap, in that order
    # line 4: avionics stream, machine
    # line 3: vicon stream, machine
    # line 2  last command executed
    # line 1: interactive command in buffer (ymax-1)

    # main loop, print,parse arduino output and send command
    start_ts = datetime.datetime.now()
    flag_calibrated = False
    flap_new_plot_data = True
    flag_ping_in_progress = False
    flag_vicon_not_ready = True

    calibration_data = []
    R_12 = None
    scale_12 = None

    epoch = datetime.datetime.now()
    debugdata = []
    serial_buffer = ""
    while not(quit):

        global curve00,curve01,curve10,curve11, ptr, data00,data01,data10
        # show loop freq
        #screen.addstr(ymax-1,xmax-20,"loop freq = "+str(int(1.0/(datetime.datetime.now()-start_ts).total_seconds())))
        screen.addstr(ymax-1,xmax-20,"ts = "+str(int((datetime.datetime.now()-epoch).total_seconds())))
        # read from avionics serial (thru xbee)
        # there's a large timeout (0.1s) so we can read before all data has arrived
        if (avionics.in_waiting > 0 ):
            data = avionics.read(8)
            msgType = data[0]
            if (msgType == 1):
                # normal update
                voltage = unpack('H',data[1:3])[0]/100.0
                flapPWM = unpack('H',data[3:5])[0]
                throttlePWM = unpack('H',data[5:7])[0]
                isTelemCtrl = data[7]

                lock_avionics_state.acquire()
                avionics_state = voltage,flapPWM,throttlePWM,isTelemCtrl
                lock_avionics_state.release()
                displayLineno(4,"%f V, F: %d, T: %d, %d"%(voltage,flapPWM,throttlePWM,isTelemCtrl))
            elif (msgType ==2):
                # ping response
                tac = time()
                flag_ping_in_progress = False
                if tic is None:
                    displayLineno(2, "Error.. Unsolicited ping response")
                else:
                    displayText("[avionics]: ping success, dt = " + str(tac-tic)+"s")
                    tic = 0.0

        # Vicon Display
        # NOTE disable vicon does not actually disable the daemon, just display
        if (enableVicon):
            lock_vicon_state.acquire()
            local_vicon_state = vicon_state
            lock_vicon_state.release()
            if (vicon_state is None):
                displayLineno(2,"Vicon unavailable")
                #enableVicon = False
                # flag indicating that "vicon not ready" is currently displayed on line 2
                flag_vicon_not_ready = True
            else:
                if (flag_vicon_not_ready):
                    flag_vicon_not_ready = False
                    displayLineno(2,"Vicon Capture Enabled")
                x,y,z,rx,ry,rz = vicon_state
                text = str(x)+","+str(y)+","+str(z)+","+str(rx)+","+str(ry)+","+str(rz)
                text = "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f"%(x,y,z,rx,ry,rz) 
                displayLineno(3,"Vicon Stream: "+text)
        else:
            displayLineno(3,"Vicon Disabled ")

        # Joystick Display
        if (enableJoystick):
            lock_js.acquire()
            local_throttle = js_throttle
            local_flap = js_flap
            lock_js.release()
            displayLineno(5,"Joystick: T: %d F: %d ,interval(ms) %d"%(local_throttle,local_flap,debug_joy_update_interval_ms))
        else:
            displayLineno(5,"Joystick Disabled")

        # Controller Output Display
        if (enableController):
            if (flag_new_remote):
                flag_new_remote = False
                displayLineno(4,"Controller: F: %d T: %d"%(remote_flapPWM,remote_throttlePWM))
        else:
            displayLineno(4,"Controller Disabled")

        screen.refresh()

        # read user input, store in buffer
        user_input = screen.getch()
        while (user_input != -1):
            command += chr(user_input)
            user_input = screen.getch()

        # display user typed command TODO add curser, editing
        # currently no backspace allowed
        screen.addstr(ymax-1,0,"Command: "+command)
        #screen.refresh()

        # process command when return is sent
        # command here includes a trailing \n, remove that before parsing
        if (len(command)>0 and command[-1] == '\n'):
            curses.beep() #doesn't seem to work
            if (command[:-1] == 'quit') or (command[:-1] == 'q'):
                quit = True
            elif command[:-1] == 'clear':
                screen.clear()
            elif command[:-1] == 'flush':
                # Not sure about this
                avionics.reset_input_buffer()
            elif command[:-1] == 'ping':
                flag_ping_in_progress = True
                tic = time()
                #avionics.write("ping\r\n".encode())
                ping_packet = bytearray(5)
                # message type
                ping_packet[0] = 2
                outcount = avionics.write(ping_packet)
                displayLineno(2,"Ping...")
            elif command[:-1] == 'vicon':
                enableVicon = not enableVicon
                if (enableVicon):
                    displayLineno(2,"Vicon Capture Enabled")
                else:
                    displayLineno(2,"Vicon Capture Disabled")
            elif command[:-1] == 'log':
                # toggle logging, start a new file each time
                if (isLogging):
                    # logging was enabled, now stopping logging
                    isLogging = False
                    lock_logfile.acquire()
                    with open(logFilename, 'a') as filehandle:
                        for entry in logBuffer:
                                filehandle.write('%s' % entry)
                    logBuffer = []
                    lock_logfile.release()
                    displayLineno(2,"logfile saved: "+logFilename)
                else:
                    no = 1
                    logBuffer = []
                    while os.path.isfile(logFolder+logPrefix+str(no)+logSuffix):
                        no += 1
                    logFilename = logFolder+logPrefix+str(no)+logSuffix
                    isLogging = True
                    displayLineno(2,"log started: "+logFilename)
            elif command[:-1] == 'joy':
                # toggle joystick
                if enableJoystick:
                    enableJoystick = False
                    thread_joystick.join()
                    displayLineno(2,"Joystick Disabled")
                else:
                    try:
                        js = pygame.joystick.Joystick(0)
                        js.init()
                        enableJoystick = True
                        thread_joystick = threading.Thread(name="joystickUpdate",target=joystickUpdateDaemon,args=(js,avionics))
                        thread_joystick.start()
                        displayLineno(2,"Joystick Enabled : "+str(js.get_name()))
                    except pygame.error as e:
                        displayLineno(2,"Joystick Not available: " + str(e))
            elif command[:-1] == 'ctrl':  # toggle external controller
                if (enableJoystick): # disable controller
                    enableJoystick = False
                    displayText("External controller off...")
                else:
                    displayText("Switch to external controller...")
                    if enableJoystick:
                        enableJoystick = False
                        thread_joystick.join()
                        displayText("Joystick Automatically Disabled")
                    vicon_list = []
                    enableJoystick = True

            else:
                screen.move(ymax-2,0)
                screen.clrtoeol()
                screen.addstr(ymax-2,0,"Unrecognized command: "+command)

            # clear user input
            screen.move(ymax-1,0)
            screen.clrtoeol()
            screen.addstr(ymax-1,0,"Command: ")
            screen.refresh()
            command = ""
        screen.refresh()

        # display new data

        if (enablePlot and flap_new_plot_data):
            if (flag_calibrated):
                # update plot (avionics related)

                data00[:-1,:] = data00[1:,:]                      # shift data in the temporal mean 1 sample left
                #data01[:-1,:] = data01[1:,:]                          # shift data in the temporal mean 1 sample left
                data10[:-1,:] = data10[1:,:]                          # shift data in the temporal mean 1 sample left
                #data11[:-1,:] = data11[1:,:]                      # shift data in the temporal mean 1 sample left

                # x axis
                data00[-1,0] = avionics_ts/1000.0 # in second
                # y axis
                #data00[-1,1] = np.linalg.norm(acc1-R_12*acc2)              # vector containing the instantaneous values      
                data00[-1,1] = 0

                # x axis
                data10[-1,0] = avionics_ts/1000.0 # in second
                # y axis
                #data00[-1,1] = np.linalg.norm(acc1-R_12*acc2)              # vector containing the instantaneous values      
                data10[-1,1] = 0


                #data10[-1,0] = testStand_ts/1000.0 # in second
                # mag angle dot product
                #data10[-1,1] = np.dot(np.array([0,1,0]),mag/np.linalg.norm(mag))

                # estimated omega from avionics
                #data10[-1,1] = kf_omega

                #data11[-1,0] = testStand_ts/1000.0 # in second
                # angle diff
                #data11[-1,1] = min(abs(int(kf_azimuth) - int(azimuth)),360-abs(int(kf_azimuth) - int(azimuth)))
                #data11[-1,1] = kf_omega - omega

                #screen.addstr(ymax-5,0,str(data01[-1,1]))
                screen.refresh()
                curve00.setData(data00)                     # set the curve with this data
                #curve01.setData(data01)                     # set the curve with this data
                curve10.setData(data10)                     # set the curve with this data
                #curve11.setData(data11)                     # set the curve with this data
                QtGui.QApplication.processEvents()    # you MUST process the plot now
            flap_new_plot_data = False


if __name__ == '__main__':
    #establish serial comm to teststand and avionics,XXX not sure how to determine order, just plug them in in order...
    vi = Vicon()
    pygame.display.init()
    pygame.joystick.init()

    host_system = platform.system()
    # automatically identifies Linux or MacOS
    if host_system == "Linux":
        avionicsCommPort = '/dev/ttyUSB0'
        #avionicsCommPort = '/dev/ttyACM0'
    elif host_system == "Darwin":
        testStandCommPort = '/dev/tty.wchusbserial1420'
        avionicsCommPort = '/dev/tty.SLAB_USBtoUART'

    try:
        if (enablePlot):
            app = QtGui.QApplication([]) 

            win = pg.GraphicsWindow(title="Avionics Feed") # create a window
            plot00 = win.addPlot(title="Phase",row=0,col=0,labels={'left':"Phase(deg)",'bottom':"Time(s)"})  # creates empty space for the plot in the window
            #plot01 = win.addPlot(title="mag",row=0,col=1,labels={'left':"ref",'bottom':"Time(s)"})  # creates empty space for the plot in the window
            plot10 = win.addPlot(title="Omega",row=1,col=0,labels={'left':"rev/s",'bottom':"Time(s)"})  # creates empty space for the plot in the window
            #plot11 = win.addPlot(title="omega difference",row=1,col=1,labels={'left':"d_theta(deg)",'bottom':"Time(s)"})  # creates empty space for the plot in the window
            curve00 = plot00.plot()                        # create an empty "plot" (a curve to plot)
            #curve01 = plot01.plot()                        # create an empty "plot" (a curve to plot)
            curve10 = plot10.plot()                        # create an empty "plot" (a curve to plot)
            #curve11 = plot11.plot()                        # create an empty "plot" (a curve to plot)
            #plot11.setYRange(0,180)

            windowWidth = 200                       # width of the window displaying the curve
            data00 = np.vstack([np.linspace(0,0,windowWidth),np.linspace(0,0,windowWidth)]).T          # create array that will contain the relevant time series     
            #data00 = np.linspace(0,0,windowWidth)
            data01 = np.vstack([np.linspace(0,0,windowWidth),np.linspace(0,0,windowWidth)]).T          # create array that will contain the relevant time series     
            data10 = np.vstack([np.linspace(0,0,int(windowWidth/2)),np.linspace(0,0,int(windowWidth/2))]).T          # create array that will contain the relevant time series     
            data11 = np.vstack([np.linspace(0,0,int(windowWidth/2)),np.linspace(0,0,int(windowWidth/2))]).T          # create array that will contain the relevant time series     
            ptr = -windowWidth                      # set first x position

            # prepare post experiment data
            mag_offset = []



        # start main update loop
        try:
            with serial.Serial(avionicsCommPort,115200, timeout=0.1) as avionics:
                # start vicon update loop
                thread_vicon = threading.Thread(name="viconUpdate",target=viconUpdateDaemon,args=(vi,avionics))
                thread_vicon.start()

                curses.wrapper(main,avionics)
        except serial.serialutil.SerialException as e:
            print(e)
    finally:
        quit = True
        thread_vicon.join()
        if (enableJoystick):
            thread_joystick.join()

        if (enablePlot):
            print("Please close the plot window")
            pg.QtGui.QApplication.exec_()

