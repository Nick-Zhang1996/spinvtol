import serial
from time import sleep

avionicsCommPort = '/dev/ttyUSB0'

with serial.Serial(avionicsCommPort,115200, timeout=0.1) as avionics:
    while True:
        if(avionics.in_waiting>0):
            serial_in = avionics.readline()
            #decoded = serial_in.decode()
            #print(decoded)
            print(serial_in)
        else:
            sleep(0.000001)
