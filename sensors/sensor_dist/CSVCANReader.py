import numpy as np
import pandas as pd

#use "ui time stamp"
time_stamp_type='ui_time_stamp'
time_mul=10**3
#time_mul=10**0

#TODO make a read function
class objects:
    def __init__(self):
        self.sensor = "CSVCAN"
        self.timer = None
        self.v = 100
        self.yaw_rate = None
        self.something = None
        #dict for nama output col name
        self.val = {"vel":None, "yaw_rate":None, "something":None}

class CSVCAN:
    def __init__(self, file_name = None, pos="all"):

        assert file_name != None
        self.file = pd.read_csv(file_name)
        self.row = 0

       
    def read(self):
        objects = []
        obj = objects()
        self.row += 1

        obj.val["timer"] = self.file.at[row, "timer"]
        obj.val["vel"] = self.file.at[row, "vel"]
        obj.val["yaw_rate"] = self.file.at[row, "yaw_rate"]
        objects.append(obj)
        return obj

        

    def get_length(self):
        #read file once and accumulate new number of them and return observed number
        tar_array = []
        for row in range(self.file_length):
            ids = self.file.at[row,"velocity"]
            if ids not in tar_array:
                tar_array.append(ids)
        return sorted(tar_array), len(tar_array)


                

if __name__ =="__main__":
    can = CSVCAN("../data/2017.07.04_at_08.10.34_export.csv")
    data = can.read()
    print(data[0].val["x"])

    data = can.read()
    print(data[0].val["x"])
