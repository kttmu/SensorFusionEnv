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



class Ref:
    def __init__(self, file_name = None, shift=[]):

        assert file_name != None
        self.file = pd.read_csv(file_name)
        self.file_length = len(self.file)
        self.row_counter = 1
        self.update_count_S=False
        self.timer = 0
        self.timer_pre = 0
        self.timer_pre_pre = 0
        self.objects = []
        #TODO get max number of observation
        #self.obs_list, self.cols = self.get_length()
        self.pos = "center"
        #self.shift = 1.3625
        self.shift = shift
        self.sensor = "Reference"

    def read(self):

        self.objects = []
        #print(self.file.at[10, "timer"])
        time_stamp =  int(self.file.at[self.row_counter, "timer"] / time_mul)
        obj_x = 0.0 

        if self.timer == 0:
            self.timer = time_stamp
            self.timer_pre = time_stamp
            self.timer_pre_pre = time_stamp
            #self.row_counter = 1
        else:
            self.timer_pre_pre = self.timer_pre
            self.timer_pre = self.timer
        
        if self.file_length > self.row_counter:
            while (self.timer == self.timer_pre):

                obj = objects()
                obj.timer =  time_stamp
                #ダサいので書き換えたい、、
                #for object
                obj.sensor  = self.file.at[self.row_counter, 'sensor']
                obj.timer   = self.file.at[self.row_counter, 'timer']
                obj.x       = self.file.at[self.row_counter, 'tar_x'] + self.shift[0]
                obj.y       = self.file.at[self.row_counter, 'tar_y'] + self.shift[1]
                obj.w       = self.file.at[self.row_counter, 'tar_w']
                obj.l       = self.file.at[self.row_counter, 'tar_l']
                obj.o       = -self.file.at[self.row_counter, 'tar_theta']#orientation
                obj.cls     = self.file.at[self.row_counter, 'tar_class']
                #obj.id     = self.file.at[self.row_counter, 'obs_id']
                obj.ref_x   = self.file.at[self.row_counter, 'tar_x'] + self.shift[0]
                obj.ref_y   = self.file.at[self.row_counter, 'tar_y'] + self.shift[1]
                obj.ref_w   = self.file.at[self.row_counter, 'tar_w']
                obj.ref_l   = self.file.at[self.row_counter, 'tar_l']
                obj.ref_cls = self.file.at[self.row_counter, 'tar_class']
                obj.ref_o   = -self.file.at[self.row_counter, 'tar_theta']#orientation 
                obj.status   = self.file.at[self.row_counter, 'status']
                obj.state = np.array([[obj.x],[obj.y],[0],[0],[obj.w],[obj.l]])

                #for dict
                obj.val["sensor"]  = self.file.at[self.row_counter, 'sensor']
                obj.val["timer"]   = self.file.at[self.row_counter, 'timer']
                obj.val["x"]       = self.file.at[self.row_counter, 'obs_x'] + self.shift[0]
                obj.val["y"]       = self.file.at[self.row_counter, 'obs_y'] + self.shift[1]
                obj.val["w"]       = self.file.at[self.row_counter, 'obs_w']
                obj.val["l"]       = self.file.at[self.row_counter, 'obs_l']
                obj.val["o"]       = self.file.at[self.row_counter, 'obs_yaw']#orientation
                obj.val["cls"]     = self.file.at[self.row_counter, 'obs_class']
                #obj.val["id"]     = self.file.at[self.row_counter, 'obs_id']
                obj.val["ref_x"]   = self.file.at[self.row_counter, 'tar_x'] + self.shift[0]
                obj.val["ref_y"]   = self.file.at[self.row_counter, 'tar_y'] + self.shift[1]
                obj.val["ref_w"]   = self.file.at[self.row_counter, 'tar_w']
                obj.val["ref_l"]   = self.file.at[self.row_counter, 'tar_l']
                obj.val["ref_cls"] = self.file.at[self.row_counter, 'tar_class']
                obj.val["ref_o"]   = self.file.at[self.row_counter, 'tar_theta'] + np.pi#orientation

                #print("readable result:", obj.ref_id, obj.sensor, obj.x,obj.ref_x, obj.status)
                obj.y = -obj.y
                obj.ref_y = -obj.ref_y

                obj = self.shifter(obj)

                obj.r = np.sqrt(obj.x ** 2.0 + obj.y ** 2.0)
                obj.theta = np.arctan2(obj.y, obj.x)
                obj.ref_r = np.sqrt(obj.ref_x ** 2.0 + obj.ref_y ** 2.0)
                obj.ref_theta = np.arctan2(obj.ref_y, obj.ref_x)
                #print(obj.r)
                #print("kaitahazu")

                if abs(self.file.at[self.row_counter, 'tar_x'] and obj_x!=obj.x and obj.ref_cls!="None") > 0: 
                    self.objects.append(obj)
                obj_x = obj.x

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
            ids = self.file.at[row,""]
            if ids not in tar_array:
                tar_array.append(ids)
        return sorted(tar_array), len(tar_array)
    
    def shifter(self,obj):
        obj.x = obj.x - obj.l*np.cos(obj.o)*0.5 - obj.w*np.cos(obj.o+np.pi/2)*0.5
        obj.y = obj.y + obj.l*np.sin(obj.o)*0.5 - obj.w*np.sin(obj.o+np.pi/2)*0.5 #特例
        obj.ref_x = obj.ref_x - obj.ref_l*np.cos(obj.ref_o)*0.5
        obj.ref_y = obj.ref_y + obj.ref_l*np.sin(obj.ref_o)*0.5 #特例
        obj.o = -obj.o
        obj.ref_o = -obj.ref_o
        return obj


                

if __name__ =="__main__":
    sur = Reference("../result/annotation_toshow.csv")
    data = sur.read()
    print(data[0].val["x"])

    data = sur.read()
    print(data[0].val["x"])
