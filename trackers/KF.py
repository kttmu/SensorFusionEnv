import numpy as np
from numpy import cos, sin, arctan2, pi
import trackers.Struck import objects



class objects:
    def __init__(self):
        self.x = None
        self.y = None
        self.v = None
        self.a = None
        self.yaw = None
        self.yawrate = None
        self.w = None
        self.l = None


class KF:
    def __init__(self):

        #IMM model
        self.vec_merge = np.zeros((8,1))
        self.vec_cv = np.zeros((8,1))
        self.vec_ctrv = np.zeros((8,1))
        self.vec_ca = np.zeros((8,1))
        self.vec_ctra = np.zeros((8,1))
        self.vec_rm = np.zeros((8,1))

        self.p_meerge = np.zeros((8,8))
        self.p_cv = np.zeros((8,8))
        self.p_ctrv = np.zeros((8,8))
        self.p_ca = np.zeros((8,8))
        self.p_ctra = np.zeros((8,8))
        self.p_rm = np.zeros((8,8))



    #TODO consider mass product argorithm, update process
    def cv(self, vec, dt):
        #cv model predict
        vec_ = np.copy(vec)
        vec_[0] = vec[0] + vec[2] * dt * cos(vec[5]) #x
        vec_[1] = vec[1] + vec[2] * dt * sin(vec[5]) #y
        vec_[2] = vec[2] 
        vec_[3] = 0.0
        vec_[4] = vec[4]
        vec_[5] = 0.0
        vec_[6] = vec[6]
        vec_[7] = vec[7]

        return vec_
