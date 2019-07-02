# data processor for test stand and monocopter avionics
import serial
import curses
from curses.textpad import Textbox, rectangle
import datetime
import time


def main(screen, ser):
    curses.curs_set(0)                      # Set cursor visibility where 0 = invisible,
    screen.nodelay(True)                    # Set getch() to be non-blocking - important!
    curses.noecho()
    curses.use_default_colors()             # Lets your terminal client set the fonts and colors
    ymax, xmax = screen.getmaxyx()          # Get the screen dimensions
    screen.setscrreg(0,ymax-3)
    screen.scrollok(True)

    command = ""
    quit = False
    # main loop, print,parse arduino output and send command
    while not(quit):
        #now = str(datetime.datetime.now())
        #screen.addstr(3, 1, now)
        #screen.refresh()

        # read from serial 
        line = ser.readline()
        if (len(line)>0):
            # append data to display, remove trailing \r\n which mess up curser
            screen.scroll()
            screen.addstr(ymax-3,0,line.decode()[:-2])
            # attempt to parse data
            try:
                # sample parsing, need more work TODO
                # remove \r\n 
                data = [float(i) for i in line[:-2].split(b',')]
                if (len(data)==3):
                    ts = data[0]
                    azimuth = data[1]
                    omega = data[2] 
                    #print(ts,azimuth,omega)
            except ValueError:
                # this is not a machine readable line, no big deal
                pass

        # read user input, store in buffer
        user_input = screen.getch()
        while (user_input != -1):
            command += chr(user_input)
            user_input = screen.getch()

        # display user typed command TODO add curser, editing
        screen.addstr(ymax-1,0,"Command: "+command)
        screen.refresh()

        # process command when return is sent
        # command here includes a trailing \n, remove that before parsing
        if (len(command)>0 and command[-1] == '\n'):
            curses.beep() #doesn't seem to work
            if command[:-1] == 'quit':
                quit = True
            else:
                # it's ok to send the trailing \n, default behavior or arduino IDE's serial monitor
                ser.write(command.encode())
                screen.move(ymax-2,0)
                screen.clrtoeol()
                screen.addstr(ymax-2,0,"Command Sent: "+command)
                # clear user input
                screen.move(ymax-1,0)
                screen.clrtoeol()
                screen.addstr(ymax-1,0,"Command: ")
                screen.refresh()
            command = ""


if __name__ == '__main__':
    #establish serial comm to teststand
    with serial.Serial('/dev/ttyUSB0',38400, timeout=0.05) as ser:
        # TODO check if serial is successfully opened

        curses.wrapper(main,ser)        
