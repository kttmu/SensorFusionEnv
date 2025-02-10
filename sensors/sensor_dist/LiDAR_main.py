import numpy as np
import cv2
import os
import struct
from read_lidar        import ReadLidar
from _udp_cap_data_read import udpDataReader

ReadLidar = ReadLidar()
LiDAR = udpDataReader(1)

TT=0

class LiDAR_contena:
    
    def __init__(self):

        self.pos = np.empty((0,4))
        self.pos_all = np.empty((0,4))
        self.cicle_count= 0
        self.OneCicleData = 38*2
        self.timestamp=0
        self.timestamp_r=0
        self.pos_r= []
        self.pos_T= 0
        self.pos_T_count= 0
        self.end=False

    def VLP_S(self,FileLidar,TimeCAN,timer,timer1):
        
        if timer==0:
            roop=True
        else:
            print("timer1", timer1)
            print("lidar time stamp:", self.timestamp)
            roop=(self.timestamp<=timer1)
            #TODO remove here right now
            #roop=True
        if roop:
            #print("A=",TimeCAN-self.timestamp)
            self.pos_r=self.pos
            self.timestamp_r=self.timestamp
            while True:
                while True:
                    d = FileLidar.read(1218)
                
                    flg, dataPacket = ReadLidar.dataPacketOut(d)
                    
                    if flg == 1:
                        
                        break
                    else:
                        while True:
                            d = FileLidar.read(1)
                            flg, dataPacket = ReadLidar.dataPacketOut(d)
                            #print("reading", LiDAR.timeStamp_xavier)
                            
                            if flg == 1:
                                break
                        
                        break
                data_Packet_=struct.pack("B"* len(dataPacket),*dataPacket)
                LiDAR.read(data_Packet_)
                X = LiDAR.X
                Y = LiDAR.Y
                Z = LiDAR.Z
                I = LiDAR.I
                self.timestamp = LiDAR.timeStamp_xavier
                lidar_cicle=self.cicle_count%self.OneCicleData
                
                #if timer == timer1:
                #    lidar_cicle = True

                if LiDAR.recv == 1:

                    self.cicle_count += 1
                
                if lidar_cicle:

                    if self.timestamp>timer1:
                        print("C=",TimeCAN-self.timestamp)
                        print("lidar:", self.timestamp)
                        print("sur:", timer1)
                        self.pos = np.zeros((X.shape[0],4))
                        self.pos[:,0]=X[:]
                        self.pos[:,1]=Y[:]
                        self.pos[:,2]=Z[:]
                        self.pos[:,3]=I[:]

                        break
                    else:
                        self.pos = np.zeros((X.shape[0],4))
                        self.pos[:,0]=X[:]
                        self.pos[:,1]=Y[:]
                        self.pos[:,2]=Z[:]
                        self.pos[:,3]=I[:]

                        self.pos_r=self.pos
                        self.timestamp_r=self.timestamp
                        #print("B=",TimeCAN-self.timestamp)
                        #print("V=",timer1-self.timestamp)

                        if self.pos_T==TimeCAN-self.timestamp:
                            self.pos_T_count += 1
                        if self.pos_T_count==30:
                            self.end=True
                            break
                        self.pos_T= TimeCAN-self.timestamp

        return self.pos_r,self.end,self.timestamp_r
        
    
           
