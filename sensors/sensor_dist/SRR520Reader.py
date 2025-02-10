import linecache
import pathlib
import numpy as np
import pandas as pd

time_stamp_type='ui_time_stamp'
time_mul=10**3

#TODO make a read function
class objects:
    def __init__(self):
        self.sensor = "SRR520"
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
        self.val = {"x":None, "y":None, "z":None, "vx":None, "vy":None, "vz":None, "w":None, "l":None, "h":None, "id":None, "timer":None, "cls":None, "o":None, "life_time":None}
        #FOR annotation tool
        self.match = False
        self.pos = None


class SRR520:
    def __init__(self, file_name = None, shift=[]):

        self.time_sur = 0
        assert file_name != None
        self.file = pd.read_csv(file_name)
        self.file_length = len(self.file)
        self.row_counter = 0
        self.update_count_S=False
        self.timer = 0
        self.timer_pre = 0
        self.objects = []
        self.obs_list, self.cols = self.get_length()
        #self.pos = pos
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

                obj.timer  = int(self.timer)
                obj.counter  = self.file.at[self.row_counter, "ui_cycle_counter"]
                obj.sensor  = self.file.at[self.row_counter, "e_sensor_type"]
                obj.x  = self.file.at[self.row_counter, "f_dist_x"] + self.shift[0]
                obj.y  = self.file.at[self.row_counter, "f_dist_y"] + self.shift[1]
                obj.vx  = self.file.at[self.row_counter, "f_vrel_x"]
                obj.vy  = self.file.at[self.row_counter, "f_vrel_y"]
                w_tmp  = self.file.at[self.row_counter, "f_width_left"]
                obj.w  = self.file.at[self.row_counter, "f_width_right"] + w_tmp
                l_tmp  = self.file.at[self.row_counter, "f_length_front"]
                obj.l  = self.file.at[self.row_counter, "f_length_rear"] + l_tmp
                obj.o  = self.file.at[self.row_counter, "f_orientation"]
                obj.cls = self.file.at[self.row_counter, "e_classification"]
                obj.rcs  = self.file.at[self.row_counter, "f_rcs"]
                obj.id  = self.file.at[self.row_counter, " u_id"]
                obj.prob  = self.file.at[self.row_counter, "f_probability_of_existence"]
                #obj.pos = self.pos
               
                obj.val["timer"]  = int(self.timer)
                obj.val["counter"]  = self.file.at[self.row_counter, "ui_cycle_counter"]
                obj.val["sensor"]  = self.file.at[self.row_counter, "e_sensor_type"]
                obj.val["x"]  = self.file.at[self.row_counter, "f_dist_x"] + self.shift[0]
                obj.val["y"]  = self.file.at[self.row_counter, "f_dist_y"] + self.shift[1]
                obj.val["vx"]  = self.file.at[self.row_counter, "f_vrel_x"]
                obj.val["vy"]  = self.file.at[self.row_counter, "f_vrel_y"]
                obj.val["w"]  = self.file.at[self.row_counter, "f_width_right"] + w_tmp
                obj.val["l"]  = self.file.at[self.row_counter, "f_length_rear"] + l_tmp
                obj.val["o"]  = self.file.at[self.row_counter, "f_orientation"]
                obj.val["cls"] = self.file.at[self.row_counter, "e_classification"]
                obj.val["rcs"]  = self.file.at[self.row_counter, "f_rcs"]
                obj.val["id"]  = self.file.at[self.row_counter, " u_id"]
                obj.val["prob"]  = self.file.at[self.row_counter, "f_probability_of_existence"]

                #shift point to center
                obj = self.shifter(obj)

                if obj.cls == 1: 
                    self.objects.append(obj)

                self.row_counter += 1
                self.timer_pre = self.timer
                self.timer =  int(self.file.at[self.row_counter, time_stamp_type].split("(")[-1].split(")")[0])
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
        if obj.x > 0:
            obj.x = obj.x + obj.l*np.cos(obj.o)*0.5
            obj.y = obj.y - obj.l*np.sin(obj.o)*0.5
        else:
            obj.x = obj.x - obj.l*np.cos(obj.o)*0.5
            obj.y = obj.y + obj.l*np.sin(obj.o)*0.5
        return obj

if __name__ =="__main__":
    sur = SRR520("../data/SRR520_Front_Left_Sensor_Object.csv", pos="FL")
    data = sur.read()
    data = sur.read()
    print(sur.pos)
    print(data[0].val["timer"])
