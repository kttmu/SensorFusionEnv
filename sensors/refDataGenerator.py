#from sensor_dist.vehile import VehicleCAN
import numpy as np
import csv
#import CAN
from sensors.SensorReader import SensorReader as Sensor



#TODO main
class refDataGenerator:
    def __init__(self, args, sensor_list = {}, xtss_mode = "latest", target="ARS510"):

        self.args = args
        self.sensor_data = {} #sensor obj dict
        self.sensor_list = sensor_list
        self.data = {} #accumulate last 5-step data
        self.save_time = 5
        self.Initialize(sensor_list)

        self.global_timer = []
        self.xtss_mode = xtss_mode #[ latest, nearest, step] 
        self.target = target

        #set global timer sensor
        #self.target_data = self.sensor_data[target]
        self.wo_target_sensor = [ids for ids in self.sensor_data]
        self.wo_target_sensor.remove(target)

        #try : 
        #    del self.sensor_data[target]
        #except:
        #    print("no target sensor mode")

    def Initialize(self, sensor_list):
        for s_id in sensor_list:
            #file_name = self.args.data_root + self.args.data_name +  sensor_list[s_id][0]
            file_name = self.args.data_root + self.args.data_name +  sensor_list[s_id]
            #self.sensor_data[s_id] = Sensor(sensor_list[s_id][0], s_id, sensor_list[s_id][1])
            self.sensor_data[s_id] = Sensor(file_name, s_id)


        norm = s_id
        tmp = self.sensor_data[s_id].read()
        #print(tmp)
        #latest_timer = tmp[-1].timer
        latest_timer = self.sensor_data[s_id].timer
        lat_sensor = s_id
        self.data[s_id] = []

        #adjust start position
        for s_id in sensor_list:
            if s_id != norm:
                self.data[s_id] = []
                data = self.sensor_data[s_id].read()
                self.data[s_id].append(data)
                tmp_timer = data[-1].timer
                #print(data[-1].timer)
                if tmp_timer > latest_timer:
                    latest_timer = tmp_timer
                    lat_sensor = s_id
                    tmp_data = data
        
        print("the latest : ", lat_sensor, latest_timer)

        for s_id in sensor_list:
            if s_id != lat_sensor:
                print("setteing", s_id,"...")
                if s_id == "LiDAR":
                    data = self.sensor_data[s_id].skip(latest_timer)
                    self.data[s_id].append(data)
                else:
                    while True:
                        data = self.sensor_data[s_id].read()
                        self.data[s_id].append(data)

                        while(len(self.data[s_id]) > self.save_time):
                            self.data[s_id].pop(0)

                        #TODO change timer trigger to class  objects
                        #print("data shape:", len(data))
                        #timer = data[-1].timer
                        timer = self.sensor_data[s_id].timer
                        #timer = self.data[s_id].timer
                        #print(timer)
                        if timer > latest_timer:
                            break

        print("initialize the position")
        #print(self.data)

                

    #=====================================#
    # function : GenerateCSV
    # input : savefile path
    # return : nothing
    #=====================================#
    def GenerateCSV(self, outputs, save_path, start=0, end=2000):

        filename = "exPA_xtss="+self.xtss_mode+"_target="+self.target+"_start="+str(start)+"s_end="+str(end)+"s.csv"

        with open(save_path + filename, mode="w", newline='') as f:
            writer = csv.writer(f)

            #=====title call=====#
            s_index = {}
            writedown = ["timer"]
            #ideal name :  SVC220::Front::obj[13].x, SVC220::Front::obj[15].x
            for s_id in self.sensor_data:
                s_index[s_id] = {}
                for obj_id in self.sensor_data[s_id].obs_list:
                    s_index[s_id][obj_id] = {}
                    for out_id in outputs:
                        col_name = s_id + "::" + self.sensor_data[s_id].pos + "::obj[" + str(obj_id) + "]." + str(out_id)
                        writedown.append(col_name)
                        s_index[s_id][obj_id][out_id] = len(writedown) - 1
                        
            writer.writerow(writedown)
            writedown = [0] * len(writedown)

            #=====data call=====#
            global_timer, data = self.read_sensors()
            init_timer = global_timer
            #TODO loop in range of minimist file
            while True:
                timer = round((global_timer - init_timer) / 10**3.0, 2)
                if timer >= start:
                    writedown[0] = timer
                    for s_id in data:
                        for i in range(len(data[s_id])):
                            obj_id = data[s_id][i].id
                            for out_id in outputs:
                                ids = s_index[s_id][obj_id][out_id]
                                writedown[ids] = data[s_id][i].val[out_id]

                    #write down to specified csv file
                    writer.writerow(writedown)
                global_timer, data = self.read_sensors()
                if timer > end:
                    break
        print("csv file named :"+filename+" was generated")






    #===================================#
    # function : readsensors
    # input : nothing
    # return : timer, tmp_data = {"sensor0": [obj, obj, obj], "sensor1": [obj, obj, obj]}
    #===================================#
    def read_sensors(self):
        #xtss method
        #----------------- Nearest ------------------------#
        if self.xtss_mode == "Nearest":

            data = {}

            #init global timer
            self.data[self.target].append(self.sensor_data[self.target].read())
            #self.data[self.target].append(self.target_data.read())
            if self.data[self.target][-1] == False:
                return False, False
            #global_timer = self.data[self.target][-1][-1].timer
            global_timer = self.sensor_data[self.target].timer_pre
            data[self.target] = self.data[self.target][-1]


            for s_id in self.wo_target_sensor:
            #for s_id in self.sensor_data:

                #timer_pre = self.data[s_id][-1][-1].timer
                timer_pre = self.sensor_data[s_id].timer_pre_pre
                #timer_pre = self.sensor_data[s_id].data.timer_pre
                
                #Read until over read
                if timer_pre <= global_timer:
                    if s_id == "LiDAR":
                        self.data[s_id].append(self.sensor_data[s_id].skip(global_timer))
                        #timer_pre = self.data[s_id][-1][0].timer_pre
                        timer_pre = self.sensor_data[s_id].timer_pre
                    else:
                        while(timer_pre <= global_timer):
                            self.data[s_id].append(self.sensor_data[s_id].read())
                            if len(self.data[s_id][-1]) > 0:
                                #timer_pre = self.data[s_id][-1][0].timer_pre
                                timer_pre = self.sensor_data[s_id].timer_pre

                #Compare timestamp
                #if len(self.data[s_id][-1]) > 0 and len(self.data[s_id][-2]) > 0:
                #    #timer     = self.data[s_id][-1][0].timer
                #    #timer_pre = self.data[s_id][-2][0].timer
                timer     = self.sensor_data[s_id].timer_pre
                timer_pre = self.sensor_data[s_id].timer_pre_pre
                diff      = abs(timer - global_timer)
                diff_     = abs(timer_pre - global_timer)
                #elif len(self.data[s_id][-1]) > 0:
                #    #timer     = self.data[s_id][-1][0].timer
                #    timer     = self.sensor_data[s_id].timer
                #    diff      = abs(timer - global_timer)
                #    diff_     = 100000
                ##elif len(self.data[s_id][-1]) == 0:
                #else:
                #    while len(self.data[s_id][-1]) == 0:
                #        del self.data[s_id][-1]
                #    timer     = self.data[s_id][-1][0].timer
                #    diff      = abs(timer - global_timer)
                #    diff_     = 100000



                if diff < diff_:
                    flg = -1
                else:
                    diff = diff_
                    flg = -2
                    timer = timer_pre
                
                #Pop old data  (required python ver > 3.7)
                while(len(self.data[s_id]) > self.save_time):
                    self.data[s_id].pop(0)

                #Extract onetime data
                if diff < 100:
                    data[s_id] = self.data[s_id][flg]
                else:
                    data[s_id] = []
            return global_timer, data
        #--------------------------------------------------#

        #----------------- Latest ------------------------#
        elif self.xtss_mode == "Latest":

            data = {}

            #init global timer
            self.data[self.target].append(self.sensor_data[self.target].read())
            #global_timer = self.data[self.target][-1][-1].timer
            global_timer = self.data[self.target][-1][-1].timer
            data[self.target] = self.data[self.target][-1]


            for s_id in self.wo_target_sensor:

                timer_pre = self.data[s_id][-1][-1].timer
                
                #Read until over read
                if timer_pre <= global_timer:
                    while(timer_pre <= global_timer):
                        self.data[s_id].append(self.sensor_data[s_id].read())
                        timer_pre = self.data[s_id][-1][0].timer

                ##Compare timestamp(pop latest data)
                #timer     = self.data[s_id][-1][0].timer
                #timer_pre = self.data[s_id][-2][0].timer
                #diff      = abs(timer - global_timer)
                #diff_     = abs(timer_pre - global_timer)

                #if diff < diff_:
                #    flg = -1
                #else:
                #    flg = -2
                #    timer = timer_pre
                
                ##Pop old data  (required python ver > 3.7)
                #while(len(self.data) > self.save_time):
                #    self.data[s_id].pop(0)

                #Extract onetime data
                data[s_id] = self.data[s_id][-2]
            return global_timer, data
        #--------------------------------------------------#

            
        #elif self.xtss_mode == "Latest":

        #    data = {}
        #    flg = 0

        #    for s_id in self.sensor_data:

        #        sensor = self.sensor_data[s_id]
        #        timer_pre = self.latest_data[s_id][-1].timer
        #        data[s_id] = []

        #        while(timer_pre < self.global_timer[-1]):
        #            tmp_ = sensor.read()
        #            data[s_id].append(tmp_)
        #            timer_pre = tmp_.timer
        #        
        #        #Replace  (required python ver > 3.7)
        #        if len(self.data) > self.save_time:
        #            self.data[s_id].pop(0)
        #        self.data[s_id][self.global_timer] = data[s_id]

        #        #Extract onetime data
        #        data[s_id] = data[s_id][flg]


        #    return global_timer, data



if __name__ == "__main__":

    file_list = {
        "SVC220":("2017.07.04_at_08.10.34_export.csv","ALL"),
        "SRR520_RL":("SRR520_Rear_Left_Sensor_Object.csv","RL"),
        "SRR520_FR":("SRR520_Front_Right_Sensor_Object.csv","FR"),
        "ARS510":("ARS510_Sensor_Object.csv","FC"),
        #"LiDAR":("lidar_center_32.bin","center"),
        "VehicleCAN":("can_vicle_1.bin","car")
        #"None":"filename.csv"
        }
    outputs = ["timer","x","y" ]
    #outputs = ["timer","x","y","vx","vy","w","l","o","prob","cls" ]

    #Data = DataIntegrator(file_list, xtss_mode="Nearest", target="SVC220", time_round=True)
    #Data = DataIntegrator(file_list, xtss_mode="Latest", target="ARS510")
    Data = refDataGenerator(file_list, xtss_mode="Nearest", target="ARS510")
    timer, sata = Data.read_sensors()
    for s_id in sata:
        print("s_id",s_id, "timer:",sata[s_id][-1].timer)
        print("s_id",s_id, "length:",len(sata[s_id]))
    for i in range(700):
        timer, sata = Data.read_sensors()
    for s_id in sata:
        print("s_id",s_id, "timer:",sata[s_id][-1].timer)
    print("vel:",sata["VehicleCAN"][0].val["v"], sata["VehicleCAN"][0].val["yawrate"])
    #Data.GenerateCSV(outputs, save_path = "./result/", start=10, end=100)
