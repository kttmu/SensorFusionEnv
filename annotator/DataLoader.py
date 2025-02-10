import numpy as np
from refDataGenerator import refDataGenerator
from point_plot import Plot

class Loader:
    def __init__(self, args):
        file_list = {
            "SVC220":("scene20-all.csv","ALL"),
            #"SRR520_RL":("SRR520_Rear_Left_Sensor_Object.csv","RL"),
            #"SRR520_RR":("SRR520_Rear_Right_Sensor_Object.csv","RR"),
            #"SRR520_FR":("SRR520_Front_Right_Sensor_Object.csv","FR"),
            #"SRR520_FL":("SRR520_Front_Left_Sensor_Object.csv","FL"),
            "ARS510":("ARS510_Sensor_Object.csv","FC"),
            "LiDAR":("lidar_center_32.bin","center"),
            #"IMX490":("video.h264","center"),
            "VehicleCAN":("can_vicle_1.bin","car")
            #"None":"filename.csv"
        }
        
        self.target = "ARS510"
        self.mode = "Nearest"
        self.Data = refDataGenerator(file_list, xtss_mode=self.mode, target=self.target)
        self.args = args
        self.vis_scale = args.vis_scale
        self.viewer = Plot()
        self.counter = 0

    def move_to_start_position(self):
        #TODO in this function
        # move to start position
        # visualize lidar data on image
        # trans form the object data 
        start_pos = self.args.crop_start
        self.init_timer, self.data = self.Data.read_sensors()
        self.timer = self.init_timer
        
        while((self.timer - self.init_timer) / 10**3.0 <  start_pos):
            print("self timer:",self.timer, (self.timer - self.init_timer) / 10**3.0, start_pos)
            self.timer, self.data = self.Data.read_sensors()

        #frame = self.data["IMX490"][0].frame
        speed_v = self.data["VehicleCAN"][0].v
        lidar_pos = self.Data.sensor_data["LiDAR"].data.clouds

        #TODO vis
        _, _, _, img_scale = self.viewer.plot_cluster([],[],[],lidar_pos,[],[],[],[],[],
                                            [],[],[],[],[],speed_v,self.vis_scale)

        #TODO return data
        #return img_scale, self.data, 1, self.timer, frame
        return img_scale, self.data, 1, self.timer

    def move_to_next_position(self, step_width):
        #TODO in this function
        # move to start position
        # visualize lidar data on image
        # trans form the object data 
        start_pos = self.args.crop_start
        self.init_timer = self.timer
        self.timer, self.data = self.Data.read_sensors()
        self.timer = self.init_timer
        
        while((self.timer - self.init_timer) / 10**3.0 <  step_width):
            print("self timer:",self.timer)
            self.timer, self.data = self.Data.read_sensors()

        #frame = self.data["IMX490"][0].frame
        #print(frame)
        speed_v = self.data["VehicleCAN"][0].v
        lidar_pos = self.Data.sensor_data["LiDAR"].data.clouds

        #TODO vis
        _, _, _, img_scale = self.viewer.plot_cluster([],[],[],lidar_pos,[],[],[],[],[],
                                            [],[],[],[],[],speed_v,self.vis_scale)

        #TODO return data
        #return img_scale, self.data, 1, self.timer, frame
        return img_scale, self.data, 1, self.timer
       
