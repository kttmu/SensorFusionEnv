import numpy as np
import numpy.linalg as LA
from numpy.linalg import inv
import pandas
from associators.Struct import objects
#from Struct import objects
from numpy import sqrt, cos, sin, pi, arctan2


class HighLevelFuser:
    def __init__(self):
        print("initialize fuser...")

    #============================================#
    #function : compute
    #input : data dict
    #output : an array of original data with "fused" objects
    #============================================#
    def compute(self, data = {}, m_noise={}):
        #TODO in this function
        #1. read data,and judge
        #2. call covariance matrix
        #3. add data with coeff matrix
        #4. break loop, and fuse the added data -> derive integrated data
        #5. return data and covariance matrix
        fused_state = np.zeros((6,1))
        fused_cov = np.zeros((6,6))
        ref_state = np.zeros((6,1))
        flg = 0 #detect flag
        

        for s_id in data:
            data_ = data[s_id]

            for ids in range(len(data_)):
                #read
                state = np.zeros((6,1))
                
                state[0,0] = data_[ids].x
                state[1,0] = data_[ids].y
                state[2,0] = data_[ids].vx
                state[3,0] = data_[ids].vy
                state[4,0] = data_[ids].w
                state[5,0] = data_[ids].l

                #s_cov = self.pullcov(s_id, data_[ids])
                s_cov = m_noise[s_id]
                fused_state += inv(s_cov) @ state
                fused_cov += inv(s_cov)

                #detect flag
                flg = 1
        
        #fuse data
        if flg == 1:
            fused_cov = inv(fused_cov)
            fused_state = fused_cov @ fused_state
        
            obj = objects()
            obj.x = fused_state[0,0]
            obj.y = fused_state[1,0]
            obj.vx = fused_state[2,0]
            obj.vy = fused_state[3,0]
            obj.w = fused_state[4,0]
            obj.l = fused_state[5,0]
            obj.cov = fused_cov

            data["fused"] = [obj]

        return data
