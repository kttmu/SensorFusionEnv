import struct
import sys
import io
import numpy as np
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

from ctypes          import *
from sensors.sensor_dist._read_vlp16_udp import VLP16
from sensors.sensor_dist._read_vlp32_udp import VLP32


class udpDataReader:
    def __init__(self,ID=0):
        self.data = b'\x00'
        self.dataPacket = b'\x00'
        self.recv = 0
        self.ID = ID
        if ID == 0:
            self.VLP16 = VLP16()
            self.X      = 0
            self.Y      = 0
            self.Z      = 0
            self.I      = 0
            self.timeStamp = 0
            self.cntr      = 0
        if ID == 1:
            self.VLP32 = VLP32()
            self.X      = 0
            self.Y      = 0
            self.Z      = 0
            self.I      = 0
            self.timeStamp = 0
            self.timeStamp_xavier =0
            self.cntr      = 0
        
    def read(self,dataPack):
        self.data = dataPack
        if   self.ID==0:
                    if (len(self.data) == 1218):
                        X,Y,Z,I,timeStamp = self.VLP16.ReadLidar(self.data)
                        self.cntr += 1 
                        #if (self.cntr%2 == 0) :
                        self.X = X
                        self.Y = Y
                        self.Z = Z
                        self.I = I
                        self.timeStamp = timeStamp
                        self.recv = 1
        
        if   self.ID==1:
                    if (len(self.data) == 1218):
                        X,Y,Z,I,timeStamp,timeStamp_xavier = self.VLP32.ReadLidar(self.data)
                        self.cntr += 1 
                        #if (self.cntr%2 == 0) :
                        self.X = X
                        self.Y = Y
                        self.Z = Z
                        self.I = I
                        self.timeStamp = timeStamp
                        self.timeStamp_xavier = timeStamp_xavier
                        self.recv = 1

# ================================================================= #


