import numpy as np
from numpy import sin, cos, tan, arctan2, pi
import pandas as pd
import cv2
import copy

#  """" add sensor class if you need """"" #
from sensors.sensor_dist.ARS510Reader import ARS510
from sensors.sensor_dist.SRR520Reader import SRR520
from sensors.sensor_dist.LiDARReader import LiDAR
from sensors.sensor_dist.SVC220Reader import SVC220
from sensors.sensor_dist.IMX490Reader import IMX490
from sensors.sensor_dist.VehicleCANReader import VehicleCAN
from sensors.sensor_dist.ReferenceReader import Reference
from sensors.sensor_dist.RefReader import Ref
#from sensor_class.HiresMAPReader import HiresMAPReader
#from sensor_dist.SonarReader import SonarReader

#""""""""""""""""""""""""""""""""""""""""""#


#data_dir = "../sur_came/raw_data/20210617_morning/scene20/"

class SensorReader:
    def __init__(self, file_name, sensor_type):

        #set sensor data
        self.sensor_type = sensor_type
        self.file_name = file_name
        self.video_time = None
        self.save_time = 5

        #set sensor data as a class
        self.data = None
        self.initialize()
        #self.obs_list = self.data.obs_list
        #self.pos = self.data.pos
        #self.tmp_timer = self.data.timer
        self.timer = self.data.timer
        self.timer_pre = self.data.timer

        assert self.data != None


    def initialize(self):
        print(self.sensor_type)
        # =========== RADAR ======================== #
        if self.sensor_type == "ARS510":
            shift = [0.882, 0]#delta_x, delta_y
            self.data = ARS510(self.file_name, shift)
        elif self.sensor_type == "SRR520_FL":
            shift = [0.643, 0.809]#delta_x, delta_y
            self.data = SRR520(self.file_name, shift)
        elif self.sensor_type == "SRR520_FR":
            shift = [0.643, -0.809]#delta_x, delta_y
            self.data = SRR520(self.file_name, shift)
        elif self.sensor_type == "SRR520_RL":
            shift = [-3.348, 0.695]#delta_x, delta_y
            self.data = SRR520(self.file_name, shift)
        elif self.sensor_type == "SRR520_RR":
            shift = [-3.348, -0.695]#delta_x, delta_y
            self.data = SRR520(self.file_name, shift)
        # ========================================== #

        # ======= CAN ============================== #
        elif self.sensor_type == "VehicleCAN":
            self.data = VehicleCAN(self.file_name)
        elif self.sensor_type == "ADASCAN":
            pass
        # ========================================== #

        # ===== Camera ============================== #
        elif self.sensor_type == "IMX490":
            self.data = IMX490(self.file_name)
        # ========================================== #


        # ============= SONAR  ======================#
        elif self.sensor_type == "SONAR":
            pass
            #self.data = SonarReader(self.file_name)
        # ========================================== #


        # =============== SVC  ===================== #
        elif self.sensor_type == "SVC220":
            self.data = SVC220(self.file_name)
        # ========================================== #


        # =============== LiDAR ===================== #
        elif self.sensor_type == "LiDAR":
            shift = [-1.695, 0.0]#delta_x, delta_y
            self.data = LiDAR(self.file_name, shift)
        # =========================================== #

        # =============== Reference =====================#
        elif self.sensor_type == "Reference":
            self.data = Reference(self.file_name)
        elif self.sensor_type == "Ref":
            if self.file_name == "../raw_data/20210804_morning/scene1/referene.csv":
                shift = [-1.695 ,0.0]
            else:
                #shift = [0,0]
                shift = [-1.695+1.34 ,0.0]
            self.data = Ref(self.file_name, shift)
        # =========================================== #


        # =============== HiresMAP ===================== #
        elif self.sensor_type == "HiresMAP":
            pass
        # =========================================== #
        
        #TDDO parse data_dir
        print("filename : ", self.file_name)
        assert self.data != None, "cannot find target sensor class... if you need, put them in './data/ '"

    def read(self):
        #print(self.data.read().timer)
        data = self.data.read() 
        self.timer_pre_pre = copy.copy(self.timer_pre)
        self.timer_pre = copy.copy(self.timer)
        self.timer = self.data.timer
        return data
        
    def skip(self, gtimer=-1):
        if self.sensor_type == "LiDAR":
            self.timer_pre_pre = copy.copy(self.timer_pre)
            self.timer_pre = copy.copy(self.timer)
            data = self.data.skip(gtimer)
            self.timer = self.data.timer
            return data

if __name__ == "__main__":
    print("succeed")
