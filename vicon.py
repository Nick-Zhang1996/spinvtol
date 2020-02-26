# Parse Vicon UDP Object Stream, single object
import socket
from time import time,sleep
from struct import unpack
from math import degrees,radians
import threading
from threading import Lock
from signal import signal, SIGINT

quit = False
lock_quit = Lock()

vicon_state = None
lock_vicon_state = Lock()

def exitHandler(signal_received, frame):
    global quit
    # Handle any cleanup here
    quit = True
    print("waiting for thread1 to finish")
    t.join()
    print("Exiting...")
    exit(0)

def viconUpateDaemon(vi):
    global quit,lock_vicon_state,vicon_state
    while (not quit):
        local_state = vi.getViconUpdate()
        #print("daemon "+str(local_state))
        lock_vicon_state.acquire()
        vicon_state = local_state
        lock_vicon_state.release()
    return

class Vicon:
    def __init__(self,IP=None,PORT=None):
        if IP is None:
            IP = "0.0.0.0"
        if PORT is None:
            PORT = 51001
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #self.sock.settimeout(0.05)
        self.sock.bind((IP, PORT))
        
    def __del__(self):
        self.sock.close()

    # NOTE this function is expected to be called very frequently, at a rate higher than Vicon's update rate (100-300Hz)
    def getViconUpdate(self):
        data, addr = self.sock.recvfrom(512)
        frameNumber = unpack('i',data[0:4])
        #print(frameNumber)
        itemsInBlock = data[4]
        itemID = data[5]
        itemDataSize = unpack('h',data[6:8])
        itemName = data[8:32].decode("ascii")
        # raw data in mm, convert to m
        x = unpack('d',data[32:40])[0]/1000
        y = unpack('d',data[40:48])[0]/1000
        z = unpack('d',data[48:56])[0]/1000
        # euler angles,rad, rotation order: rx,ry,rz, using axis of intermediate frame
        rx = unpack('d',data[56:64])[0]
        ry = unpack('d',data[64:72])[0]
        rz = unpack('d',data[72:80])[0]
        #print(x,y,z,degrees(rx),degrees(ry),degrees(rz))
        return (x,y,z,rx,ry,rz)

    def testFreq(self,packets=100):
        # test actual frequency of vicon update, with PACKETS number of state updates
        tic = time()
        for i in range(packets):
            self.getViconUpdate()
        tac = time()
        return packets/(tac-tic)


    # for debug
    def fromFile(self,filename):
        newFile = open(filename, "wb")
        newFile.write(data)


if __name__ == '__main__':
    #f = open('samplevicon.bin','br')
    #data = f.read()
    vi = Vicon()
    tik = time()
    last_frame = None
    loss_count = 0
    # example: for very simple uses where it's possible to loop at high frequency
    while False:
        print(vi.getViconUpdate())

    # example Test Frequency
    # test freq
    if False:
        for i in range(3):
            print("Freq = "+str(vi.testFreq())+"Hz")

    # example: if main loop is run at a frequency lower than vicon update rate
    # getViconUpdate() should be called at high frequency in a separate thread
    # this example utilizes viconUpdateDaemon, some global variables, and an exit Handler
    if True:
        signal(SIGINT,exitHandler)
        t = threading.Thread(name="vicon",target=viconUpateDaemon,args=(vi,))
        t.start()
        for i in range(20):
            lock_vicon_state.acquire()
            print("Main Thread : "+str(vicon_state))
            lock_vicon_state.release()
            sleep(1)

        
    

