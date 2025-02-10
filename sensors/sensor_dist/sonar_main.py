import sys
import cv2
import struct
import os
import csv
import math
import time
import numpy as np

from read_can          import ReadCAN

ReadCAN   = ReadCAN()

TT=0
class CanBus:
    def __init__(self):
        
        self.timestamp_ = 0
        self.speed_     = 0
        self.yawrate_   = 0

        self.PSM_Object_Info1_ = []
        self.PSM_Object_Info2_ = []
        self.PSM_Object_Info3_ = []

        self.timestamp_r = 0
        self.speed_r     = 0
        self.yawrate_r   = 0

        self.PSM_Object_Info1_r_ = []
        self.PSM_Object_Info2_r_ = []
        self.PSM_Object_Info3_r_ = []
        self.sonar_up=False

    def CAN(self,FileCan,timer1,timer):

        if timer==0:
            roop=True
        else:
            roop=(self.timestamp_<=timer1)
        
        if roop:
            self.PSM_Object_Info1_r_.clear()
            self.PSM_Object_Info2_r_.clear()
            self.PSM_Object_Info3_r_ .clear()
            self.PSM_Object_Info1_r_ = self.PSM_Object_Info1_
            self.PSM_Object_Info2_r_ = self.PSM_Object_Info2_
            self.PSM_Object_Info3_r_ = self.PSM_Object_Info3_
            self.speed_r = self.speed_
            self.yawrate_r = self.yawrate_
            self.timestamp_r = self.timestamp_

            self.PSM_Object_Info1_.clear()
            self.PSM_Object_Info2_.clear()
            self.PSM_Object_Info3_ .clear()

            if timer!=0:
                self.sonar_up=True

            while True:
               
                while True:
                        d = FileCan.read(1)
                    
                        TimeCAN, flg, speed, yawrate,PSM_Object_Info1,PSM_Object_Info2, PSM_Object_Info3, PSM_Distance_Info= ReadCAN.CANOut(d)
                    
                        
                        if flg == 1:
                            if TimeCAN>timer1:
                                self.yawrate  = yawrate
                                self.speed = speed
                            else:
                                self.speed_r = speed
                                self.yawrate_r = yawrate

                        if flg == 1 and len(PSM_Object_Info1)==8 and len(PSM_Object_Info3)==8 and len(PSM_Object_Info2)==8:
                        
                        
                            break
                if TimeCAN > timer1:
                    
                    self.PSM_Object_Info1_ = PSM_Object_Info1
                    self.PSM_Object_Info2_ = PSM_Object_Info2
                    self.PSM_Object_Info3_ = PSM_Object_Info3
                    self.speed_ = speed
                    self.yawrate_  = yawrate
                    self.timestamp_ = TimeCAN
                    
                    break
                
                else:

                    self.PSM_Object_Info1_r_.clear()
                    self.PSM_Object_Info2_r_.clear()
                    self.PSM_Object_Info3_r_ .clear()

                    self.PSM_Object_Info1_r_ = PSM_Object_Info1
                    self.PSM_Object_Info2_r_ = PSM_Object_Info2
                    self.PSM_Object_Info3_r_ = PSM_Object_Info3
                    self.speed_r = speed
                    self.yawrate_r = yawrate
                    self.timestamp_r = TimeCAN
                    self.sonar_up=True
                    
        else:
            self.sonar_up=False
            self.PSM_Object_Info1_r_ = []
            self.PSM_Object_Info2_r_ = []
            self.PSM_Object_Info3_r_ = []
        

                    
        return self.timestamp_r, self.speed_r,self.yawrate_r,self.PSM_Object_Info1_r_,self.PSM_Object_Info3_r_,self.PSM_Object_Info2_r_,self.sonar_up
        
