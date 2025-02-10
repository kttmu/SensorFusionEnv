import sys
import cv2
import struct
import os
import csv
import math
import time
import numpy as np

from sensors.sensor_dist.read_can_v          import ReadCAN


TT=0
time_mull = 10 ** 3.0

class objects:
    def __init__(self):
        self.v = None
        self.yawrate = None
        self.val = {"v":None, "yaw_rate":None,"timer":None}

class VehicleCAN:
    def __init__(self, file_name = None):
        
        self.timer = 0
        self.timer_pre = 0
        self.timer_pre_pre = 0
        self.file = open(file_name, 'rb')
        self.ReadCAN   = ReadCAN()
        self.pos = "car"
        self.obs_list = []
        self.cols = []
        self.objects = []
        #self.file = open(FileName+'can_vicle_1.bin', 'rb' 


    def read(self):
    #def CAN(self,FileCan,timer1,timer):
        #self.objects = []
        while True:
            d = self.file.read(1)
            
            TimeCAN, flg, speed, yawrate= self.ReadCAN.CANOut(d)
            #print(speed)

            if flg == 1 :
            
                break
        obj = objects()
        obj.v = speed
        obj.timer = int(TimeCAN / time_mull)
        obj.yawrate = yawrate

        obj.val["v"] = speed
        obj.val["timer"] = int(TimeCAN / time_mull)
        obj.val["yawrate"] = yawrate
        self.objects = [obj]
        self.timer_pre_pre = self.timer_pre
        self.timer_pre = self.timer
        self.timer = obj.timer
        return self.objects

        ##TODO reverse "=" ,if  problem occured
        #if TimeCAN >= timer1:
        #    
    
        #    self.speed_ = speed
        #    self.yawrate_  = yawrate
        #    self.timestamp_ = TimeCAN
        #    
        #    break
        
        #else:

        #    
        #    self.speed_r = speed
        #    self.yawrate_r = yawrate
        #    self.timestamp_r = TimeCAN
        #     
        

        #            
        #return self.timestamp_r, self.speed_r,self.yawrate_r
        
