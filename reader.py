#!/usr/bin/python3
# data processor for test stand and monocopter avionics
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

# from bottom, how far up do we display rolling data
rolling_offset = 10

def main(screen, testStand, avionics):
    curses.curs_set(0)                      # Set cursor visibility where 0 = invisible,
    screen.nodelay(True)                    # Set getch() to be non-blocking - important!
    curses.noecho()
    curses.use_default_colors()             # Lets your terminal client set the fonts and colors
    ymax, xmax = screen.getmaxyx()          # Get the screen dimensions
    screen.setscrreg(0,ymax-rolling_offset)
    screen.scrollok(True)

    command = ""
    quit = False
    # ---------- display format ------------
    # --- human readable text ----- (scrollable,non-erasing)
    # line 4: avionics output, machine
    # line 3: test stand output, machine
    # line 2 last command executed
    # line 1: interactive command in buffer

    # main loop, print,parse arduino output and send command
    start_ts = datetime.datetime.now()
    flag_calibrated = False
    flag_new_data_testStand = False
    flag_new_data_avionics = True

    calibration_data = []
    R_12 = None
    scale_12 = None

    epoch = datetime.datetime.now()
    while not(quit):

        global curve,curve1,curve2, ptr, Xm,Ym,Zm
        # show loop freq
        #screen.addstr(ymax-1,xmax-20,"loop freq = "+str(int(1.0/(datetime.datetime.now()-start_ts).total_seconds())))
        screen.addstr(ymax-1,xmax-20,"ts = "+str(int((datetime.datetime.now()-epoch).total_seconds())))

        # read from test stand serial 
        if (testStand.in_waiting>20):
            line = testStand.readline()
            # machine readable data start with #
            if (chr(line[0])=='#'):
                # display data, remove trailing \r\n which mess up curser
                screen.move(ymax-3,0)
                screen.clrtoeol()
                screen.addstr(ymax-3,0,"Test Stand Datastream: "+line.decode()[:-2])
                # attempt to parse data
                try:
                    # sample parsing, need more work TODO
                    # remove prefix #, suffix \r\n 
                    data = [float(i) for i in line[1:-2].split(b',')]
                    if (len(data)==3):
                        testStand_ts = data[0]
                        azimuth = data[1]
                        # omega unit rev/s
                        omega = data[2] 
                        flag_new_data_avionics = True
                        #print(ts,azimuth,omega)
                except ValueError:
                    # this is not a machine readable line, no big deal
                    pass
            else:
                #human readable
                screen.scroll()
                try:
                    text = line.decode()[:-2]
                    screen.addstr(ymax-rolling_offset,0,"[test stand]: "+text)
                except UnicodeDecodeError:
                    pass
            screen.refresh()

        # read from avionics serial (thru xbee)
        # BUG XXX: reading too fast
        # a complete message has at least 62 bytes
        if (avionics.in_waiting > 75 ):
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
                    if (len(data)==11):
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

                        # rev/s from avionics
                        custom = data[10] 
                        try:
                            if (custom and omega is not None and azimuth is not None and omega>1):
                                mag_offset.append([omega, azimuth])
                                screen.move(ymax-6,0)
                                screen.clrtoeol()
                                screen.addstr(ymax-6,0,"Data count: "+str(len(mag_offset)))
                        except NameError: # in case omega is not ready
                            pass
                        flag_new_data_testStand = True
                        #screen.addstr(ymax-9,0,"acc1 = "+str(acc1))
                        #screen.addstr(ymax-9,int(xmax/2),"acc2 = "+str(acc2))
                except ValueError:
                    # this is not a machine readable line, no big deal
                    pass
            else:
                #human readable
                screen.scroll()
                # may throw UnicodeDecodeError, not big deal
                try:
                    text = line.decode()[:-2]
                    screen.addstr(ymax-rolling_offset,0,"[avionics]: "+text)
                except UnicodeDecodeError:
                    pass
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
                testStand.reset_input_buffer()
                avionics.reset_input_buffer()
            elif command[:-1] == 'sync':
                screen.scroll()
                screen.addstr(ymax-rolling_offset,0,"[reader]: "+"Automatic Sync sequence")

                testStand.write('sync\r\n'.encode())
                screen.scroll()
                screen.addstr(ymax-rolling_offset,0,"Command Sent[test stand]: sync")
                curses.napms(100)

                avionics.write('sync\r\n'.encode())
                epoch = datetime.datetime.now()
                screen.scroll()
                screen.addstr(ymax-rolling_offset,0,"Command Sent[avionics]: sync")
                screen.refresh()

            elif command[:-1] == 'calibrate':
                flag_calibrated = False
                calibration_data = []
                screen.addstr(ymax-2,0,"calibrationg start")
                screen.refresh()

            else:
                # command start with single letter 't': intended for teststand
                # e.g.  "t sync" -> send "sync" to teststand (with additional trailing \r\n)
                # similarly 'a' for avionics
                # it's ok to send the trailing \n, default behavior or arduino IDE's serial monitor
                if command[0] == 't':
                    # test stand
                    # TODO more elegantly remove t and trailing space
                    testStand.write(command[2:].encode())
                    screen.move(ymax-2,0)
                    screen.clrtoeol()
                    screen.addstr(ymax-2,0,"Command Sent[test stand]: "+command[2:])
                elif command[0] == 'a':
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

        # data processing
        if (flag_new_data_testStand):
            # calibration
            if (not flag_calibrated):
                if (len(calibration_data)<100):
                    calibration_data.append( [acc1_x,acc1_y,acc1_z,acc2_x,acc2_y,acc2_z])
                    screen.move(ymax-2,0)
                    screen.clrtoeol()
                    screen.addstr(ymax-2,0,"Calibrating... " + str(len(calibration_data)) +"/100, do not move.")
                    screen.refresh()
                else:
                    calibration_data = np.array(calibration_data)
                    avg_acc_by_axis = np.average(calibration_data, axis=0)
                    vec_acc1 = np.matrix(avg_acc_by_axis[:3]).T
                    vec_acc2 = np.matrix(avg_acc_by_axis[3:]).T
                    # acc2 = R*acc1, this includes scaling
                    R_12 = 2*(vec_acc1+vec_acc2)*(vec_acc1+vec_acc2).T/((vec_acc1+vec_acc2).T*(vec_acc1+vec_acc2)) - np.identity(3)
                    mod1 = (avg_acc_by_axis[0]**2 + avg_acc_by_axis[1]**2 + avg_acc_by_axis[2]**2)**0.5
                    mod2 = (avg_acc_by_axis[3]**2 + avg_acc_by_axis[4]**2 + avg_acc_by_axis[5]**2)**0.5
                    screen.move(ymax-2,0)
                    screen.clrtoeol()
                    screen.addstr(ymax-2,0,"Calibration Done")
                    flag_calibrated = True

            flag_new_data_testStand = False

        if (flag_new_data_avionics):
            if (flag_calibrated):
                # update plot (avionics related)

                Xm[:-1,:] = Xm[1:,:]                      # shift data in the temporal mean 1 sample left
                Ym[:-1,:] = Ym[1:,:]                          # shift data in the temporal mean 1 sample left
                Zm[:-1,:] = Zm[1:,:]                          # shift data in the temporal mean 1 sample left
                Cm[:-1,:] = Cm[1:,:]                      # shift data in the temporal mean 1 sample left

                Xm[-1,0] = avionics_ts/1000.0 # in second
                Xm[-1,1] = np.linalg.norm(acc1-R_12*acc2)              # vector containing the instantaneous values      

                Ym[-1,0] = avionics_ts/1000.0 # in second
                if (abs(omega)>1):
                    Ym[-1,1] = Xm[-1,1]/omega**2              # vector containing the instantaneous values      
                else: 
                    Ym[-1,1] = 0

                Zm[-1,0] = testStand_ts/1000.0 # in second
                Zm[-1,1] = np.dot(np.array([0,1,0]),mag/np.linalg.norm(mag))

                Cm[-1,0] = testStand_ts/1000.0 # in second
                Cm[-1,1] = min(abs(int(custom) - int(azimuth)),360-abs(int(custom) - int(azimuth)))

                screen.addstr(ymax-5,0,str(Ym[-1,1]))
                screen.refresh()
                curve.setData(Xm)                     # set the curve with this data
                curve1.setData(Ym)                     # set the curve with this data
                curve2.setData(Zm)                     # set the curve with this data
                curve3.setData(Cm)                     # set the curve with this data
                QtGui.QApplication.processEvents()    # you MUST process the plot now
            flag_new_data_avionics = False




if __name__ == '__main__':
    #establish serial comm to teststand and avionics,XXX not sure how to determine order, just plug them in in order...

    host_system = platform.system()
    if host_system == "Linux":
        testStandCommPort = '/dev/ttyUSB0'
        avionicsCommPort = '/dev/ttyUSB1'
    elif host_system == "Darwin":
        testStandCommPort = '/dev/tty.wchusbserial1420'
        avionicsCommPort = '/dev/tty.SLAB_USBtoUART'

    try:
        app = QtGui.QApplication([]) 

        win = pg.GraphicsWindow(title="Avionics Feed") # create a window
        plot00 = win.addPlot(title="||acc1-acc2||",row=0,col=0,labels={'left':"modulus",'bottom':"Time(s)"})  # creates empty space for the plot in the window
        plot01 = win.addPlot(title="Dacc/omega**2",row=0,col=1,labels={'left':"ratio",'bottom':"Time(s)"})  # creates empty space for the plot in the window
        plot10 = win.addPlot(title="Mag angle",row=1,col=0,labels={'left':"dot",'bottom':"Time(s)"})  # creates empty space for the plot in the window
        plot11 = win.addPlot(title="angle difference",row=1,col=1,labels={'left':"d_theta(deg)",'bottom':"Time(s)"})  # creates empty space for the plot in the window
        curve = plot00.plot()                        # create an empty "plot" (a curve to plot)
        curve1 = plot01.plot()                        # create an empty "plot" (a curve to plot)
        curve2 = plot10.plot()                        # create an empty "plot" (a curve to plot)
        curve3 = plot11.plot()                        # create an empty "plot" (a curve to plot)

        windowWidth = 200                       # width of the window displaying the curve
        Xm = np.vstack([np.linspace(0,0,windowWidth),np.linspace(0,0,windowWidth)]).T          # create array that will contain the relevant time series     
        #Xm = np.linspace(0,0,windowWidth)
        Ym = np.vstack([np.linspace(0,0,windowWidth),np.linspace(0,0,windowWidth)]).T          # create array that will contain the relevant time series     
        Zm = np.vstack([np.linspace(0,0,windowWidth/2),np.linspace(0,0,windowWidth/2)]).T          # create array that will contain the relevant time series     
        Cm = np.vstack([np.linspace(0,0,windowWidth/2),np.linspace(0,0,windowWidth/2)]).T          # create array that will contain the relevant time series     
        ptr = -windowWidth                      # set first x position

        # prepare post experiment data
        mag_offset = []

        # starts main update loop

        with serial.Serial(testStandCommPort,115200, timeout=0.001) as testStand:
            with serial.Serial(avionicsCommPort,115200, timeout=0.001) as avionics:
                # TODO check if serial is successfully opened

                curses.wrapper(main,testStand,avionics)        
    finally:
        # visualize relation between omega and theta(when magnetic reading is max)
        #mag_offset = np.array(mag_offset)
        #plt.scatter(mag_offset[:,0],mag_offset[:,1])
        #plt.show()

        print("Please close the plot window")
        pg.QtGui.QApplication.exec_()

