# simultaneously record vicon feed and control signal
# vicon feed: (x,y,z,rx,ry,rz) from UDP Objectstream, float/double
# rx,ry,rx are euler angles in rad, rotation order: rx,ry,rz
# xy plane is roughly ground, z points upward
# Control: (throttle, flap) integer 1000-2000 directly from transmitter, center is around 1500

# data format: filename.log, each line is one entry, formatted as
# time(in sec), x(in meter),y,z,rx(in rad),ry,rz, throttle, flap
# line starting with # is a comment, first line is always a comment

from time import time
from vicon import Vicon
import serial
import os.path
from signal import signal, SIGINT
from sys import exit

def exitHandler(signal_received, frame):
    # Handle any cleanup here
    print("Exiting...")
    with open(logFilename, 'a') as filehandle:
        for entry in records:
                filehandle.write('%s' % entry)
    print("file saved: "+logFilename)
    exit(0)

def captureOnce():
    # flush serial
    ready = False
    while (radio.in_waiting>17):
        line = radio.readline()
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
                    ready = True
            except ValueError:
                # this is not a machine readable line, no big deal
                pass
    # a bit sketchy sync...
    if ready:
        (x,y,z,rx,ry,rz) = vi.getViconUpdate()
        dataFrame = str(time())+","+str(x)+","+str(y)+","+str(z)+","+str(rx)+","+str(ry)+","+str(rz)+","+str(throttle)+","+str(flap)+"\n"
        #dataFrame = '%d'%throttle+","+'%d'%flap+"\n"
        records.append(dataFrame)
        print(dataFrame)

if __name__ == '__main__':
    signal(SIGINT,exitHandler)

    radioReceiverCommPort = '/dev/ttyUSB0'
    logFolder = "./log/"
    logPrefix = "run"
    logSuffix = ".txt"
    no = 1
    while os.path.isfile(logFolder+logPrefix+str(no)+logSuffix):
        no += 1
    logFilename = logFolder+logPrefix+str(no)+logSuffix

    vi = Vicon()

    records = []
    with serial.Serial(radioReceiverCommPort,115200, timeout=0.001) as radio:
        print("recording... Press Ctrl-C to stop")
        while True:
            captureOnce()

            if len(records)>1000:
                with open(logFilename, 'a') as filehandle:
                    for entry in records:
                            filehandle.write('%s' % entry)
                records = []
