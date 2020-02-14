# check serial command rate
import serial
from time import time

radioReceiverCommPort = '/dev/ttyUSB0'

with serial.Serial(radioReceiverCommPort,115200, timeout=0.0033) as radio:
    count = 0
    total_count = 1000
    tic = time()
    while count < total_count:
        if (radio.in_waiting>11):
            line = radio.readline()
            if (chr(line[0])=='#'):
                # parse data
                try:
                    # remove prefix #, suffix \r\n 
                    data = [float(i) for i in line[1:-2].split(b',')]
                    if (len(data)==2):
                        throttle = data[0]
                        flap = data[1]
                        count += 1
                except ValueError:
                    # this is not a machine readable line, no big deal
                    pass
    tac = time()
    freq = total_count/(tac-tic)
    print(str(freq)+"Hz")

