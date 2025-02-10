import numpy as np
import cv2
import os
import struct
from functools import lru_cache

from sensors.sensor_dist.read_lidar        import ReadLidar
from sensors.sensor_dist._udp_cap_data_read import udpDataReader



time_mull = 10 ** 3.0

class objects:
    def __init__(self):
        self.sensor = "VLP32"
        self.timer = None
        self.x = 100
        self.y = None
        self.z = None
        self.I = None
        #dict for nama output col name
        self.val = {"x":None, "y":None, "z":None, "I":None, "timer":None}

class LiDAR:
    def __init__(self, file_name = None, shift=[]):
        self.cicle_count= 0
        self.OneCicleData = 38*2

        self.ReadLidar = ReadLidar()
        self.LiDAR = udpDataReader(1)

        #assert file_name != None
        self.sensor = "LiDAR"
        self.file = open(file_name, 'rb')
        self.objects = []
        self.objects_pre = []
        self.timer = 0
        self.obs_list = []
        #Add lidar clouds
        self.clouds = None
        self.shift = shift
        self.dataPacket = None

    def read(self):
        
        self.objects = []
        while True:
            while True:
                d = self.file.read(1218)
            
                flg, dataPacket = self.ReadLidar.dataPacketOut(d)
                
                if flg == 1:
                    
                    break
                else:
                    while True:
                        d = self.file.read(1)
                        flg, dataPacket = self.ReadLidar.dataPacketOut(d)
                        #print("reading", LiDAR.timeStamp_xavier)
                        
                        if flg == 1:
                            break
                    
                    break
            data_Packet_=struct.pack("B"* len(dataPacket),*dataPacket)
            self.LiDAR.read(data_Packet_)
            self.timer = int(self.LiDAR.timeStamp_xavier / time_mull)

            X = self.LiDAR.X
            Y = self.LiDAR.Y
            Z = self.LiDAR.Z
            I = self.LiDAR.I
            
            for i in range(len(self.LiDAR.X)):
                obj = objects()
                obj.x = self.LiDAR.X[i] + self.shift[0]
                obj.y = self.LiDAR.Y[i] + self.shift[1]
                obj.z = self.LiDAR.Z[i]
                obj.I = self.LiDAR.I[i]
                obj.timer = self.timer

                obj.val["x"] = self.LiDAR.X[i] + self.shift[0]
                obj.val["y"] = self.LiDAR.Y[i] + self.shift[1]
                obj.val["z"] = self.LiDAR.Z[i]
                obj.val["I"] = self.LiDAR.I[i]
                obj.val["timer"] = self.timer

                self.objects.append(obj)

            lidar_cicle=self.cicle_count%self.OneCicleData

            if self.LiDAR.recv == 1:
                self.cicle_count += 1
            
            if lidar_cicle:
                return self.objects

    def skip(self, gtimer):
        
        self.objects_pre = self.objects
        self.objects = []
        #while True:
        while(gtimer > self.timer):
        #while(gtimer > self.timer and not lidar_cicle):
            #print("0")
            while True:
                d = self.file.read(1218)
        
                flg, self.dataPacket = self.ReadLidar.dataPacketOut(d)
                
                if flg == 1:
                    
                    break
                else:
                    while True:
                        d = self.file.read(1)
                        flg, self.dataPacket = self.ReadLidar.dataPacketOut(d)
                        #print("reading", LiDAR.timeStamp_xavier)
                        
                        if flg == 1:
                            break
                    
                    break
            data_Packet_=struct.pack("B"* len(self.dataPacket),*self.dataPacket)
            self.LiDAR.read(data_Packet_)
            self.timer = int(self.LiDAR.timeStamp_xavier / time_mull)
        
            lidar_cicle=self.cicle_count%self.OneCicleData
            #print(lidar_cicle)

            if self.LiDAR.recv == 1:
                self.cicle_count += 1


        data_Packet_=struct.pack("B"* len(self.dataPacket),*self.dataPacket)
        self.LiDAR.read(data_Packet_)
        self.timer = int(self.LiDAR.timeStamp_xavier / time_mull)
        
        for i in range(len(self.LiDAR.X)):
            obj = objects()
            obj.x = self.LiDAR.X[i] + self.shift[0]
            obj.y = self.LiDAR.Y[i] + self.shift[1]
            obj.z = self.LiDAR.Z[i]
            obj.I = self.LiDAR.I[i]
            obj.timer = self.timer

            obj.val["x"] = self.LiDAR.X[i] + self.shift[0]
            obj.val["y"] = self.LiDAR.Y[i] + self.shift[1]
            obj.val["z"] = self.LiDAR.Z[i]
            obj.val["I"] = self.LiDAR.I[i]
            obj.val["timer"] = self.timer

            self.objects.append(obj)

        self.clouds = np.zeros((self.LiDAR.X.shape[0],4))
        self.clouds[:,0]=self.LiDAR.X[:] + self.shift[0]
        self.clouds[:,1]=self.LiDAR.Y[:] + self.shift[1]
        self.clouds[:,2]=self.LiDAR.Z[:]
        self.clouds[:,3]=self.LiDAR.I[:]

        lidar_cicle=self.cicle_count%self.OneCicleData

        #if self.LiDAR.recv == 1:
        #    self.cicle_count += 1
        
        if lidar_cicle:
            #print(lidar_cicle)
            return self.objects
        else:
            #print(lidar_cicle)
            return self.objects_pre

       
    
           
    
           
