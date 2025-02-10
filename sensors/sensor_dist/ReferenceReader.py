import numpy as np
import pandas as pd

#use "ui time stamp"
time_stamp_type='ui_time_stamp'
#time_stamp_type='xavier_system_clock'
#time_stamp_type='rcv_time'
time_mul=1
#time_mul=10**0

#TODO make a read function
class objects:
    def __init__(self):
        self.sensor = None
        self.timer = None
        self.x = None
        self.y = None
        self.w = None
        self.l = None
        self.h = None
        self.o = None #orientation
        self.cls = None
        #self.id = None
        self.ref_x = None
        self.ref_y = None
        self.ref_w = None
        self.ref_l = None
        self.ref_id = None
        self.ref_cls = None
        self.ref_o = None #orientation

        self.r = None
        self.theta = None
        self.ref_r = None
        self.ref_theta = None

        #additional 
        self.vx = None
        self.vy = None
        self.ref_vx = None
        self.ref_vy = None

        #dict for nama output col name
        self.val = {"x":None, "y":None, "z":None, "w":None, "l":None, "h":None, "timer":None, "cls":None, "o":None}
        self.match = False
        self.pos = None
        self.status = None
        self.state = 120
        self.prob = 0.0



class Reference:
    def __init__(self, file_name = None, pos="all", mode="SVC220.FC"):

        assert file_name != None
        self.file = pd.read_csv(file_name)
        self.file_length = len(self.file)
        self.row_counter = 1
        self.update_count_S=False
        self.timer = 0
        self.timer_pre = 0
        self.objects = []
        #TODO get max number of observation
        self.obs_list, self.cols = self.get_length()
        self.pos = pos
        self.shift = 1.3625
        self.sensor = "Reference"

    def read(self):

        self.objects = []
        #print(self.file)#.at[self.row_counter, "timer"])
        time_stamp =  int(self.file.at[self.row_counter, "timer"] / time_mul)

        if self.timer == 0:
            self.timer = time_stamp
            self.timer_pre = time_stamp
            #self.row_counter = 1
        else:
            self.timer_pre = self.timer
        
        if self.file_length > self.row_counter:
            while (self.timer == self.timer_pre):

                obj = objects()
                obj.timer =  time_stamp
                #ダサいので書き換えたい、、
                #for object
                obj.sensor  = self.file.at[self.row_counter, 'sensor']
                obj.timer   = self.file.at[self.row_counter, 'timer']
                obj.x       = self.file.at[self.row_counter, 'obs_x'] 
                obj.y       = self.file.at[self.row_counter, 'obs_y']
                obj.w       = self.file.at[self.row_counter, 'obs_w']
                obj.l       = self.file.at[self.row_counter, 'obs_l']
                obj.o       = self.file.at[self.row_counter, 'obs_theta']#orientation
                obj.cls     = self.file.at[self.row_counter, 'obs_class']
                #obj.id     = self.file.at[self.row_counter, 'obs_id']
                obj.ref_x   = self.file.at[self.row_counter, 'tar_x']
                obj.ref_y   = self.file.at[self.row_counter, 'tar_y']
                obj.ref_w   = self.file.at[self.row_counter, 'tar_w']
                obj.ref_l   = self.file.at[self.row_counter, 'tar_l']
                obj.ref_id  = self.file.at[self.row_counter, 'tar_id']
                obj.ref_cls = self.file.at[self.row_counter, 'tar_class']
                obj.ref_o   = self.file.at[self.row_counter, 'tar_theta']#orientation
                obj.status   = self.file.at[self.row_counter, 'status']
                obj.state = np.array([[obj.x],[obj.y],[0],[0],[obj.w],[obj.l]])

                #for dict
                obj.val["sensor"]  = self.file.at[self.row_counter, 'sensor']
                obj.val["timer"]   = self.file.at[self.row_counter, 'timer']
                obj.val["x"]       = self.file.at[self.row_counter, 'obs_x']
                obj.val["y"]       = self.file.at[self.row_counter, 'obs_y']
                obj.val["w"]       = self.file.at[self.row_counter, 'obs_w']
                obj.val["l"]       = self.file.at[self.row_counter, 'obs_l']
                obj.val["o"]       = self.file.at[self.row_counter, 'obs_theta']#orientation
                obj.val["cls"]     = self.file.at[self.row_counter, 'obs_class']
                #obj.val["id"]     = self.file.at[self.row_counter, 'obs_id']
                obj.val["ref_x"]   = self.file.at[self.row_counter, 'tar_x']
                obj.val["ref_y"]   = self.file.at[self.row_counter, 'tar_y']
                obj.val["ref_w"]   = self.file.at[self.row_counter, 'tar_w']
                obj.val["ref_l"]   = self.file.at[self.row_counter, 'tar_l']
                obj.val["ref_id"]  = self.file.at[self.row_counter, 'tar_id']
                obj.val["ref_cls"] = self.file.at[self.row_counter, 'tar_class']
                obj.val["ref_o"]   = self.file.at[self.row_counter, 'tar_theta']#orientation

                #print("readable result:", obj.ref_id, obj.sensor, obj.x,obj.ref_x, obj.status)
                obj.y = -obj.y

                obj = self.shifter(obj)

                obj.r = np.sqrt(obj.x ** 2.0 + obj.y ** 2.0)
                obj.theta = np.arctan2(obj.y, obj.x)
                obj.ref_r = np.sqrt(obj.ref_x ** 2.0 + obj.ref_y ** 2.0)
                obj.ref_theta = np.arctan2(obj.ref_y, obj.ref_x)
                #print(obj.r)
                #print("kaitahazu")

                self.objects.append(obj)

                self.row_counter += 1
                self.timer_pre = self.timer
                try:
                    self.timer =  int(self.file.at[self.row_counter, "timer"] / time_mul)
                except:
                    return False
                #print(self.timer)
        return self.objects

    def get_length(self):
        #TODO in this dunction
        #read file once and accumulate new number of them and return observed number
        tar_array = []
        for row in range(self.file_length):
            ids = self.file.at[row,"tar_id"]
            if ids not in tar_array:
                tar_array.append(ids)
        return sorted(tar_array), len(tar_array)
    
    def shifter(self,obj):
        s_id = obj.sensor
        if s_id == "SVC220.ALL" or s_id == "SVC220":
            obs_x = 0.0
            obs_y = 0.0
            ref_x = -obj.ref_l / 2 * np.cos(obj.ref_o)
            ref_y = obj.ref_l / 2 * np.sin(obj.ref_o)
        if s_id == "ARS510.FC" or s_id == "ARS510":
            if obj.x > 75:
                obs_x = obj.ref_l / 2 * np.cos(obj.ref_o)
                obs_y = obj.ref_l / 2 * np.sin(obj.ref_o)
                ref_x = -obj.ref_l / 2 * np.cos(obj.ref_o)
                ref_y = obj.ref_l / 2 * np.sin(obj.ref_o)
            else:
                obs_x = obj.l / 2 * np.cos(obj.o)
                obs_y = obj.l / 2 * np.sin(obj.o)
                ref_x = -obj.ref_l / 2 * np.cos(obj.ref_o)
                ref_y = obj.ref_l / 2 * np.sin(obj.ref_o)
        #print(s_id)
        obj.x = obj.x + obs_x
        obj.y = obj.y + obs_y
        obj.ref_x = obj.ref_x + ref_x
        obj.ref_y = obj.ref_y + ref_y
        return obj


                

if __name__ =="__main__":
    sur = Reference("../result/annotation_toshow.csv")
    data = sur.read()
    print(data[0].val["x"])

    data = sur.read()
    print(data[0].val["x"])
