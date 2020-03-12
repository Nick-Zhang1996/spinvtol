import serial
from time import sleep,time
from struct import unpack,pack
import binascii
avionicsCommPort = '/dev/ttyACM0'

outdata = bytearray(7)
# first two bytes serve as alignment, indicating start of package
outdata[0] = 9
outdata[1] = 9
# message type
outdata[2] = 2
# flap pwm
outdata[3:5] = pack('H',1500)
# throttle pwm
outdata[5:] = pack('H',1606)

ts = 0
try:
    with serial.Serial(avionicsCommPort,115200, timeout=0.1) as avionics:
        while True:
            if (avionics.in_waiting > 0):
                #print(avionics.in_waiting)
                data = avionics.read(8)
                #msgType = unpack('B',data[0])
                msgType = data[0]
                isTelemCtrl = data[1]
                voltage = unpack('H',data[2:4])[0]/100.0
                flapPWM = unpack('H',data[4:6])[0]
                throttlePWM = unpack('H',data[6:8])[0]
                #isTelemCtrl = unpack('B',data[7])
                print(msgType,voltage,flapPWM,throttlePWM,isTelemCtrl)
                # NOTE incoming: 1, 1105,1500,1707,1

            if (time()-ts>0.020):
                outcount = avionics.write(outdata)
                ts = time()

            if (False and avionics.in_waiting>0):
                line = avionics.readline()
                try:
                    print(line.decode('ascii'))
                except UnicodeDecodeError:
                    pass

            #print('out '+str(outcount)+':'+binascii.hexlify(outdata))
            sleep(0.01)
except serial.serialutil.SerialException as e:
    print(e)
