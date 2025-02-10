import numpy as np
import pandas as pd
import cv2
import pathlib

time_mull=10**3

class objects:
    def __init__(self):
        self.sensor = "IMX490"
        self.frame = None
        self.timer = None
        self.timer_pre = None
        self.timer_pre_pre = None
        #dict for name output col name
        self.val = {"frame":None, "timer":None}


class IMX490:
    def __init__(self, file_name = None, pos="center"):

        assert file_name != None
        self.file = cv2.VideoCapture(file_name) 
        fname = file_name + ".timestamps"
        #timfile = pathlib.Path("./data/", 'video.h264.timestamps')
        self.timer_file = open(fname, mode="r") 
        self.timer = 0
        self.timer_pre = 0
        self.timer_pre_pre = 0
        self.objects = []
        #TODO get max number of observation
        self.pos = pos
        #fileName_Video = pathlib.Path(FileName, 'video.h264')
        #fileName_VideoTime = pathlib.Path(FileName, 'video.h264.timestamps')
        #fileVideo     = cv2.VideoCapture(str(fileName_Video))
        #fileVideoTime = fileName_VideoTime.open(mode='r')

    def read(self):

        self.objects = []
        fret, frame = self.file.read()
        #self.timer = self.camera_.timeVideo
        video_timer = self.timer_file.readline()
        video_timer  = video_timer[:-1].split('(')

        obj = objects()
        obj.frame = frame
        self.timer_pre_pre = self.timer_pre
        self.timer_pre = self.timer
        self.timer  = (int(video_timer[1][:-1]) / time_mull)
        obj.timer = int(self.timer)
        self.objects = [obj]

        #size = (width, height)
        #im_resize = cv2.resize(frame, size, interpolation=cv2.INTER_LINEAR)
        #im_resize = frame
        return self.objects 
                

if __name__ =="__main__":
    sur = SVC220("../data/2017.07.04_at_08.10.34_export.csv")
    data = sur.read()
    print(data[0].val["x"])

    data = sur.read()
    print(data[0].val["x"])
