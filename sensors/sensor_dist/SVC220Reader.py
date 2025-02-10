import numpy as np
import pandas as pd

#use "ui time stamp"
time_stamp_type='ui_time_stamp'
#time_stamp_type='xavier_system_clock'
#time_stamp_type='rcv_time'
time_mul=10**3
#time_mul=10**0

#TODO make a read function
class objects:
    def __init__(self):
        self.sensor = "SVC220"
        self.timer = None
        self.x = 100
        self.y = None
        self.z = None
        self.id = None
        self.w = None
        self.l = None
        self.h = None
        self.vx = None
        self.vy = None
        self.vz = None
        self.o = None
        self.cls = None
        self.prob = None
        #dict for nama output col name
        self.val = {"x":None, "y":None, "z":None, "vx":None, "vy":None, "vz":None, "w":None, "l":None, "h":None, "id":None, "timer":None, "cls":None, "o":None}
        self.match = False
        self.pos = None
        self.state = None



class SVC220:
    def __init__(self, file_name = None, pos="all"):

        assert file_name != None
        self.file = pd.read_csv(file_name)
        self.file_length = len(self.file)
        self.row_counter = 0
        self.update_count_S=False
        self.timer = 0
        self.timer_pre = 0
        self.objects = []
        #TODO get max number of observation
        self.obs_list, self.cols = self.get_length()
        self.pos = pos
        self.shift = 1.3625
       
    def read(self):

        self.objects = []
        time_stamp =  int(self.file.at[self.row_counter, time_stamp_type] / time_mul)

        if self.timer == 0:
            self.timer = time_stamp
            self.timer_pre = time_stamp
            self.timer_pre_pre = time_stamp
            self.row_counter = 1
        else:
            self.timer_pre_pre = self.timer_pre
            self.timer_pre = self.timer
        
        if self.file_length > self.row_counter:
            while (self.timer == self.timer_pre):

                obj = objects()

                obj.timer =  time_stamp
                #ダサいので書き換えたい、、
                #for object
                obj.x =   self.file.at[self.row_counter, 'f_dist_x'] + self.shift
                obj.y =   self.file.at[self.row_counter, 'f_dist_y'] 
                obj.z =   self.file.at[self.row_counter, 'f_dist_z']
                obj.id =  self.file.at[self.row_counter, 'id']
                obj.vx =  self.file.at[self.row_counter, 'f_vrel_x']
                obj.vy =  self.file.at[self.row_counter, 'f_vrel_y'] / 3.6
                obj.vz =  self.file.at[self.row_counter, 'f_vrel_z'] / 3.6
                obj.w =   self.file.at[self.row_counter, 'f_width']
                obj.l =   self.file.at[self.row_counter, 'f_length']
                obj.h =   self.file.at[self.row_counter, 'f_height']
                obj.o =   self.file.at[self.row_counter, 'f_orientation']
                obj.cls = self.file.at[self.row_counter, 'e_class']
                obj.prob = self.file.at[self.row_counter, 'f_confidence']
                obj.pos = self.pos
                obj.state = np.array([[obj.x],[obj.y],[obj.vx],[obj.vy],[obj.w],[obj.l]])

                #for dict
                obj.val["timer"] = self.timer
                obj.val["x"] =   self.file.at[self.row_counter, 'f_dist_x'] + self.shift
                obj.val["y"] =   self.file.at[self.row_counter, 'f_dist_y'] 
                obj.val["z"] =   self.file.at[self.row_counter, 'f_dist_z']
                obj.val["id"] =  self.file.at[self.row_counter, 'id']
                obj.val["vx"] =  self.file.at[self.row_counter, 'f_vrel_x']
                obj.val["vy"] =  self.file.at[self.row_counter, 'f_vrel_y']
                obj.val["vz"] =  self.file.at[self.row_counter, 'f_vrel_z']
                obj.val["w"] =   self.file.at[self.row_counter, 'f_width']
                obj.val["l"] =   self.file.at[self.row_counter, 'f_length']
                obj.val["h"] =   self.file.at[self.row_counter, 'f_height']
                obj.val["o"] =   self.file.at[self.row_counter, 'f_orientation']
                obj.val["cls"] = self.file.at[self.row_counter, 'e_class']
                obj.val["prob"] = self.file.at[self.row_counter, 'f_confidence']


                self.objects.append(obj)

                self.row_counter += 1
                self.timer_pre = self.timer
                self.timer =  int(self.file.at[self.row_counter, time_stamp_type] / time_mul)
        return self.objects

    def get_length(self):
        #TODO in this dunction
        #read file once and accumulate new number of them and return observed number
        tar_array = []
        for row in range(self.file_length):
            ids = self.file.at[row,"id"]
            if ids not in tar_array:
                tar_array.append(ids)
        return sorted(tar_array), len(tar_array)


                

if __name__ =="__main__":
    sur = SVC220("../data/2017.07.04_at_08.10.34_export.csv")
    data = sur.read()
    print(data[0].val["x"])

    data = sur.read()
    print(data[0].val["x"])
