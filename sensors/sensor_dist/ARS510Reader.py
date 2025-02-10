import linecache
import pandas as pd
import numpy as np

#use "ui time stamp"
time_stamp_type='ui_time_stamp'
#time_stamp_type='xavier_system_clock'

#TODO make a read function
class objects:
    def __init__(self):
        self.sensor = "ARS510"
        self.timer = None #timestamp
        self.x = None     #x
        self.y = None     #y
        self.z = None     #z
        self.id = None    #id
        self.w = None     #width
        self.l = None     #length
        self.h = None     #height
        self.vx = None    #velocity x
        self.vy = None    #velocity y
        self.vz = None    #velocity z
        self.o = None     #orientation[rad]
        self.cls = None   #cls
        self.uid = None
        self.rcs = None
        self.life = None
        self.eangle = None
        self.val = {}
        self.pos = None
        self.match = False

class ARS510:
    def __init__(self, file_name = None, shift=[]):

        self.time_sur = 0
        assert file_name != None
        self.file = pd.read_csv(file_name)
        self.file_length = len(self.file)
        self.row_counter = 0
        self.update_count_S=False
        self.timer = 0
        self.timer_pre = 0
        self.timer_pre_pre = 0
        self.objects = []
        self.obs_list, self.cols = self.get_length()
        self.shift = shift
       
    def read(self):

        self.objects = []
        time_stamp =  self.file.at[self.row_counter, time_stamp_type].split("(")[-1].split(")")[0]

        if self.timer == 0:
            self.timer = int(time_stamp)
            self.timer_pre = int(time_stamp)
            self.timer_pre_pre = int(time_stamp)
            self.row_counter = 1
            #self.timer = 1
        else:
            self.timer_pre_pre = self.timer_pre
            self.timer_pre = self.timer
        
        if self.file_length > self.row_counter:
            while (self.timer == self.timer_pre):

                obj = objects()
                
                #for objects data
                obj.timer = self.timer
                obj.counter = self.file.at[self.row_counter,"ui_cycle_counter"]
                obj.x = self.file.at[self.row_counter,"f_dist_x"] + self.shift[0]
                obj.y = self.file.at[self.row_counter,"f_dist_y"] + self.shift[1]
                obj.vx = self.file.at[self.row_counter,"f_vrel_x"] / 3.6
                obj.vy = self.file.at[self.row_counter,"f_vrel_y"] / 3.6
                obj.w = self.file.at[self.row_counter,"f_width"]
                obj.l = self.file.at[self.row_counter,"f_length"]
                obj.o = self.file.at[self.row_counter,"f_orientation"]
                obj.cls = self.file.at[self.row_counter,"e_classification"]
                obj.rcs = self.file.at[self.row_counter,"f_rcs"]
                obj.eangle = self.file.at[self.row_counter,"f_elvation_angle"]
                obj.life = self.file.at[self.row_counter,"f_life_time"]
                obj.prob = self.file.at[self.row_counter,"f_probability_of_existence"]
                obj.id = self.file.at[self.row_counter," u_id"]
                #obj.pos = self.pos

                #for dictionary data
                obj.val["timer"] = self.timer
                obj.val["counter"] = self.file.at[self.row_counter,"ui_cycle_counter"]
                obj.val["x"] = self.file.at[self.row_counter,"f_dist_x"] + self.shift[0]
                obj.val["y"] = self.file.at[self.row_counter,"f_dist_y"] + self.shift[1]
                obj.val["vx"] = self.file.at[self.row_counter,"f_vrel_x"]
                obj.val["vy"] = self.file.at[self.row_counter,"f_vrel_y"]
                obj.val["w"] = self.file.at[self.row_counter,"f_width"]
                obj.val["l"] = self.file.at[self.row_counter,"f_length"]
                obj.val["o"] = self.file.at[self.row_counter,"f_orientation"]
                obj.val["cls"] = self.file.at[self.row_counter,"e_classification"]
                obj.val["rcs"] = self.file.at[self.row_counter,"f_rcs"]
                obj.val["eangle"] = self.file.at[self.row_counter,"f_elvation_angle"]
                obj.val["life_time"] = self.file.at[self.row_counter,"f_life_time"]
                obj.val["prob"] = self.file.at[self.row_counter,"f_probability_of_existence"]
                obj.val["id"] = self.file.at[self.row_counter," u_id"]

                #shift point to center of obj
                obj = self.shifter(obj)

                if obj.cls == 1:
                    self.objects.append(obj)

                self.row_counter += 1
                self.timer_pre = self.timer
                self.timer =  int(self.file.at[self.row_counter, time_stamp_type].split("(")[-1].split(")")[0])
                #self.timer =  self.file.at[self.row_counter, time_stamp_type]
        return self.objects


    def get_length(self):
        #TODO in this dunction
        #read file once and accumulate new number of them and return observed number
        tar_array = []
        for row in range(self.file_length):
            ids = self.file.at[row," u_id"]
            if ids not in tar_array:
                tar_array.append(ids)
        return sorted(tar_array), len(tar_array)

    def shifter(self, obj):
        obj.x = obj.x + obj.l*np.cos(obj.o)*0.5
        obj.y = obj.y - obj.l*np.sin(obj.o)*0.5
        return obj


                

if __name__ =="__main__":
    sur = ARS510("../data/ARS510_Sensor_Object.csv")
    data = sur.read()
    data = sur.read()
    print(data[0].timer)
    print(data[0].val["timer"])
