# data processor for test stand and monocopter avionics
import serial
import curses
from curses.textpad import Textbox, rectangle
import datetime
import time
import platform


def main(screen, testStand, avionics):
    curses.curs_set(0)                      # Set cursor visibility where 0 = invisible,
    screen.nodelay(True)                    # Set getch() to be non-blocking - important!
    curses.noecho()
    curses.use_default_colors()             # Lets your terminal client set the fonts and colors
    ymax, xmax = screen.getmaxyx()          # Get the screen dimensions
    screen.setscrreg(0,ymax-5)
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

    while not(quit):
        screen.addstr(ymax-1,xmax-20,str(int(1.0/(datetime.datetime.now()-start_ts).total_seconds())))
        start_ts = datetime.datetime.now()
        #now = str(datetime.datetime.now())
        #screen.addstr(3, 1, now)
        #screen.refresh()

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
                        ts = data[0]
                        azimuth = data[1]
                        omega = data[2] 
                        #print(ts,azimuth,omega)
                except ValueError:
                    # this is not a machine readable line, no big deal
                    pass
            else:
                #human readable
                screen.scroll()
                try:
                    text = line.decode()[:-2]
                    screen.addstr(ymax-5,0,"[test stand]: "+text)
                except UnicodeDecodeError:
                    pass
            #screen.refresh()

        # read from avionics serial (thru xbee)
        # BUG XXX: reading too fast
        # a complete message has at least 62 bytes
        if (avionics.in_waiting> 90 ):
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
                    if (len(data)==10):
                        # TODO complete the parsing
                        ts = data[0]
                        mx = data[1]
                        my = data[2] 
                        mz = data[3] 

                        acc1_x = data[4]
                        acc1_y = data[5] 
                        acc1_z = data[6] 

                        acc2_x = data[7]
                        acc2_y = data[8] 
                        acc2_z = data[9] 
                except ValueError:
                    # this is not a machine readable line, no big deal
                    pass
            else:
                #human readable
                screen.scroll()
                # may throw UnicodeDecodeError, not big deal
                try:
                    text = line.decode()[:-2]
                    screen.addstr(ymax-5,0,"[avionics]: "+text)
                except UnicodeDecodeError:
                    pass
            #screen.refresh()

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
            if command[:-1] == 'quit':
                quit = True
            elif command[:-1] == 'clear':
                screen.clear()
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


if __name__ == '__main__':
    #establish serial comm to teststand and avionics,XXX not sure how to determine order, just plug them in in order...

    host_system = platform.system()
    if host_system == "Linux":
        testStandCommPort = '/dev/ttyUSB0'
        avionicsCommPort = '/dev/ttyUSB1'
    elif host_system == "Darwin":
        testStandCommPort = '/dev/tty.wchusbserial1420'
        avionicsCommPort = '/dev/tty.SLAB_USBtoUART'
    with serial.Serial(testStandCommPort,38400, timeout=0.001) as testStand:
        with serial.Serial(avionicsCommPort,38400, timeout=0.04) as avionics:
            # TODO check if serial is successfully opened

            curses.wrapper(main,testStand,avionics)        
