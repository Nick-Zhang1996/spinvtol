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
import os.path

# -------  Settings ------------
formatISfullsensor = False
enablePlot = False
enableVicon = False
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

# avionics_state: voltage,flapPWM,throttlePWM,isAutomaticControl
avionics_state = None
lock_avionics_state = Lock()

lock_logfile = Lock()
quit = False
isLogging = False
logFilename = None
logBuffer = []

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

def viconUpateDaemon(vi):
    global quit,lock_vicon_state,vicon_state,avionics_state,lock_avionics_state
    global isLogging,logFilename,logBuffer,lock_logfile
    while (not quit):
        local_vicon_state = vi.getViconUpdate()
        lock_vicon_state.acquire()
        vicon_state = local_vicon_state
        lock_vicon_state.release()


        if (isLogging):
            lock_avionics_state.acquire()
            local_avionics_state = avionics_state
            lock_avionics_state.release()
            if (local_vicon_state is None):
                # should not happen
                continue
            x,y,z,rx,ry,rz = local_vicon_state
            voltage,flapPWM,throttlePWM,isAutomaticControl = local_avionics_state

            dataFrame = str(time())+","+str(x)+","+str(y)+","+str(z)+","+str(rx)+","+str(ry)+","+str(rz)+","+str(throttlePWM)+","+str(flapPWM)+","+str(voltage)+","+ str(1 if isAutomaticControl else 0) + "\n"
            logBuffer.append(dataFrame)
            if len(logBuffer)>1000:
                lock_logfile.acquire()
                with open(logFilename, 'a') as filehandle:
                    for entry in logBuffer:
                            filehandle.write('%s' % entry)
                logBuffer = []
                lock_logfile.release()

    return


def main(screen, avionics):
    global screenHandle,enableVicon,enablePlot,formatISfullsensor
    global quit,lock_vicon_state,vicon_state,avionics_state,lock_avionics_state
    global isLogging,logFilename,logBuffer,lock_logfile
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
    # line 4: avionics output, machine
    # line 3: test stand output, machine
    # line 2 last command executed
    # line 1: interactive command in buffer (ymax-1)

    # main loop, print,parse arduino output and send command
    start_ts = datetime.datetime.now()
    flag_calibrated = False
    flap_new_plot_data = True
    flag_ping_in_progress = False

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
        if (avionics.in_waiting > 0 ):
            line = avionics.readline()
            # machine readable data start with #
            if (chr(line[0])=='#'):
                # display data, remove trailing \r\n which mess up curser
                screen.move(ymax-4,0)
                screen.clrtoeol()
                screen.addstr(ymax-4,0,"Avionics Datastream: "+line.decode()[:-2])
                # attempt to parse data
                try:
                    # sample parsing, need more work TODO
                    # remove prefix #, suffix \r\n 
                    data = [float(i) for i in line[1:-2].split(b',')]
                    # two protocols, one with 13 fields
                    if (formatISfullsensor):
                        if (len(data)==13):
                            # TODO complete the parsing
                            avionics_ts = data[0]
                            mx = data[1]
                            my = data[2] 
                            mz = data[3] 
                            mag = np.matrix(data[1:4]).T

                            acc1_x = data[4]
                            acc1_y = data[5] 
                            acc1_z = data[6] 
                            acc1 = np.matrix(data[4:7]).T

                            acc2_x = data[7]
                            acc2_y = data[8] 
                            acc2_z = data[9] 
                            acc2 = np.matrix(data[7:10]).T

                            voltage = data[10]

                            # azimuth (degree) from avionics
                            kf_azimuth = data[11] 
                            # omega (rev/s) from avionics
                            kf_omega = data[12] 
                            #screen.addstr(ymax-9,0,"acc1 = "+str(acc1))
                            #screen.addstr(ymax-9,int(xmax/2),"acc2 = "+str(acc2))

                    else:   # the other protocol contains seq_no, voltage, flap, throttle, isAutomaticControl
                        if (len(data)==5):
                            seq_no = int(data[0])
                            voltage = data[1]
                            flapPWM = int(data[2])
                            throttlePWM = int(data[3])
                            isAutomaticControl = data[4]>0.5

                            lock_avionics_state.acquire()
                            avionics_state = voltage,flapPWM,throttlePWM,isAutomaticControl
                            lock_avionics_state.release()

                except ValueError:
                    pass
            elif (chr(line[0])=='$'):
                # inquiry response, e.g. ping response
                try:
                    response = line.decode()[1:-2]
                except UnicodeDecodeError:
                    pass

                displayText("[avionics response]:"+response)
                if (response == "ping"):
                    tac = time()
                    flag_ping_in_progress = False
                    if tic is None:
                        displayLineno(2, "Error.. Unsolicited ping response")
                    else:
                        displayText("[avionics]: ping success, dt = " + str(tac-tic)+"s")
                        tic = 0.0
                    
            else:
                #human readable
                screen.scroll()
                # may throw UnicodeDecodeError, not big deal
                try:
                    text = line.decode()[:-2]
                    screen.addstr(ymax-rolling_offset,0,"[avionics]: "+text)
                except UnicodeDecodeError:
                    pass
        # Vicon display
        # NOTE disable vicon does not actually disable the daemon, just display
        if (enableVicon):
            lock_vicon_state.acquire()
            local_vicon_state = vicon_state
            lock_vicon_state.release()
            if (vicon_state is None):
                displayLineno(2,"Vicon not ready")
                enableVicon = False
            else:
                x,y,z,rx,ry,rz = vicon_state
                text = str(x)+","+str(y)+","+str(z)+","+str(rx)+","+str(ry)+","+str(rz)
                text = "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f"%(x,y,z,rx,ry,rz) 
                displayLineno(3,"Vicon Stream: "+text)
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
                avionics.reset_input_buffer()
            elif command[:-1] == 'ping':
                flag_ping_in_progress = True
                tic = time()
                avionics.write("ping\r\n".encode())
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


            else:
                # command start with single letter 't': intended for teststand
                # command start with single letter 'a': intended for avionics
                # e.g.  "t sync" -> send "sync" to teststand (with additional trailing \r\n)
                # similarly 'a' for avionics
                # it's ok to send the trailing \n, default behavior or arduino IDE's serial monitor
                if command[0] == 'a':
                    # avionics
                    avionics.write(command[2:].encode())
                    screen.move(ymax-2,0)
                    screen.clrtoeol()
                    screen.addstr(ymax-2,0,"Command Sent[avionics]: "+command[2:])
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


        # start vicon update loop
        thread_vicon = threading.Thread(name="viconUpdate",target=viconUpateDaemon,args=(vi,))
        thread_vicon.start()

        # start main update loop
        with serial.Serial(avionicsCommPort,115200, timeout=0.1) as avionics:
            curses.wrapper(main,avionics)
    finally:
        quit = True
        thread_vicon.join()
        

        if (enablePlot):
            print("Please close the plot window")
            pg.QtGui.QApplication.exec_()

