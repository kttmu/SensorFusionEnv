import numpy as np
import cv2
from sensors.refDataGenerator import refDataGenerator
from annotator.point_plot import Plot
from associators.structure_tracking import Tracking_Object_List, Observed_Object_List

class Loader:
    def __init__(self, args):
        #TODO int this function
        #input data format
        #select target sensor
        #mode = Reference
        #mode = ARS510 ..and so on 

        file_list = {
            #"Reference":("reference.csv"),
            "SRR520_RL":("SRR520_Rear_Left_Sensor_Object.csv"),
            "SRR520_RR":("SRR520_Rear_Right_Sensor_Object.csv"),
            "SRR520_FR":("SRR520_Front_Right_Sensor_Object.csv"),
            "SRR520_FL":("SRR520_Front_Left_Sensor_Object.csv"),
            #"ARS510":("ARS510_Sensor_Object.csv"),
            #"LiDAR":("lidar_center_32.bin"),
            #"IMX490":("video.h264"),
            #"SVC220":("svc_export.csv"),
            "VehicleCAN":("can_vicle_1.bin"),
            "Ref":("ref.csv")
            #"None":"filename.csv"
        }
        self.target = "SRR520_FL"
        #self.target = "ARS510"
        self.mode = args.xtss
        self.Data = refDataGenerator(args, file_list, xtss_mode=self.mode, target=self.target)
        self.args = args
        self.vis_scale = args.vis_scale
        self.viewer = Plot()
        self.counter = 0

    def read(self):
        #TODO in this function
        start_pos = self.args.crop_start
        self.timer, self.data = self.Data.read_sensors()
        self.data["fused"] = []
        
        #if len(self.data["IMX490"]) > 0:
        #    print("##############")
        #    cv2.imwrite("./img.jpg", self.data["IMX490"][0].frame)

       
        lidar_pos = []
        if "LiDAR" in self.data:
            lidar_pos = self.Data.sensor_data["LiDAR"].data.clouds
            lidar = self.data.pop("LiDAR")
        speed_v = self.data["VehicleCAN"][0].v
        can = self.data.pop("VehicleCAN")
        #img = self.data.pop("IMX490")
        
        #TODO vis
        img_scale = 0
        #_, _, _, img_scale = self.viewer.plot_cluster([],[],[],lidar_pos,[],[],[],[],[],
        #                                    [],[],[],[],[],speed_v,self.vis_scale)

        #TODO return data
        #return img_scale, self.data, 1, self.timer, frame
        return img_scale, self.data, can, 1, self.timer, lidar_pos
       