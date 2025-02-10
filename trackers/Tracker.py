#from trackers.KF import KF
#from trackers.EKF import EKF
#import trackers.UKF import UKF
from trackers.IMM_EKF import IMM_EKF
from trackers.IMM_KF import IMM_KF
import numpy as np
#import trackers.IMM_UKF import IMM_UKF

class Tracker:
    def __init__(self, obs, can, mode="IMM_KF"):
        self.tracker = None
        self.SelectTracker(mode)
        self.InitTracker(obs, can)
        error_message = "Miss initialize tracking objects, checkout target class'" + mode +"' exists"
        assert self.tracker != None, error_message
        self.data = self.tracker.data
        self.prob = 0.0
        self.liklihood = 0.0
        self.judge = "hold"

    def SelectTracker(self, mode="EKF"):
        #select model
        #==== KF =================#
        if mode == "KF":
            #self.tracker = KF()
            pass
        #==== EKF =================#
        elif mode == "EKF":
            #self.tracker = EKF()
            pass
        #==== UKF =================#
        elif mode == "UKF":
            #self.tracker = UKF()
            pass
        #==== KF =================#
        elif mode == "IMM_KF":
            self.tracker = IMM_KF()
        elif mode == "KF":
            pass
            #self.tracker = IMM_KF()
        #==== EKF =================#
        elif mode == "IMM_EKF":
            self.tracker = IMM_EKF()
        #==== UKF =================#
        elif mode == "IMM_UKF":
            #self.tracker = IMM_UKF()
            pass
        
    #==========================================#
    #function : InitTracker
    #input : observed data , can data, and call init function from each tracking class
    #output : nothing
    #==========================================#
    def InitTracker(self, obs, can):
        #TODO in this function
        self.tracker.init(obs, can)

        self.x =  self.tracker.x 
        self.y  = self.tracker.y
        self.vx = self.tracker.vx 
        self.vy = self.tracker.vy 
        self.ax = self.tracker.ax 
        self.ay = self.tracker.ay 
        self.w  = self.tracker.w 
        self.l  = self.tracker.l
        #self.o  = self.tracker.o
        self.o = np.arctan2(self.tracker.vy,self.tracker.vx)

    #==========================================#
    #function : InitTracker
    #input : observed data , can data, and call init function from each tracking class
    #output : nothing
    #==========================================#
    def Update(self, can, can_pre, obs, obs_cov):
        #TODO in this function
        self.tracker.update(can, can_pre, obs, obs_cov)
        
        self.x =  self.tracker.x 
        self.y  = self.tracker.y
        self.vx = self.tracker.vx 
        self.vy = self.tracker.vy 
        self.ax = self.tracker.ax 
        self.ay = self.tracker.ay 
        self.w  = self.tracker.w 
        self.l  = self.tracker.l
        self.o  = self.tracker.o
        self.o = np.arctan2(self.tracker.vy,self.tracker.vx)

    #==========================================#
    #function : Predict
    #input : can obj(derive delta t)
    #output : nothing
    #==========================================#
    def Predict(self, can, can_pre, p_noise):
        self.tracker.predict(can, can_pre, p_noise)

        self.x =  self.tracker.x 
        self.y  = self.tracker.y
        self.vx = self.tracker.vx 
        self.vy = self.tracker.vy 
        self.ax = self.tracker.ax 
        self.ay = self.tracker.ay 
        self.w  = self.tracker.w 
        self.l  = self.tracker.l
        self.o  = self.tracker.o
        self.o = np.arctan2(self.tracker.vy,self.tracker.vx)
    



