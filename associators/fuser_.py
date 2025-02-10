
import numpy as np
import numpy.linalg as LA
from numpy.linalg import inv
import pandas
from associators.Struct import objects
#from Struct import objects
from numpy import sqrt, cos, sin, pi, arctan2


class HighLevelFuser:
    def __init__(self):
        self.std = {}
        #self.std["ARS510.FC"] = {"x":0.298,"y":0.81, "r":0.316,"theta":0.022, "w":0.1334,"l":1.193, "o":1.0}
        self.std["ARS510"] = {"x":0.298,"y":0.81, "r":0.316,"theta":0.022, "w":0.1334,"l":1.193, "o":1.0}
        self.std["SVC220_FL"] = {"x":0.298,"y":0.81, "r":0.316,"theta":0.022, "w":0.1334,"l":1.193, "o":1.0}
        self.std["SRR520_FR"] = {"x":0.298,"y":0.81, "r":0.316,"theta":0.022, "w":0.1334,"l":1.193, "o":1.0}
        self.std["SRR520_RR"] = {"x":0.298,"y":0.81, "r":0.316,"theta":0.022, "w":0.1334,"l":1.193, "o":1.0}
        self.std["SRR520_RL"] = {"x":0.298,"y":0.81, "r":0.316,"theta":0.022, "w":0.1334,"l":1.193, "o":1.0}
        #self.std["SVC220.ALL"] = {"x":4.21,"y":0.51, "r":8.22 ,"theta":0.254, "w":0.945,"l":1.514, "o":1.0}
        self.std["SVC220"] = {"x":4.21,"y":0.51, "r":8.22 ,"theta":0.254, "w":0.945,"l":1.514, "o":1.0}
        #self.std["SVC220.ALL"] = {"x":7.7,"y":0.88, "r":8.22 ,"theta":0.254, "w":0.945,"l":1.514, "o":1.0}
        #4.206296674	0.510885711	8.216388163	0.25402813	0.945377827	1.541418214

    #============================================#
    #function : compute
    #input : data dict
    #output : an array of original data with "fused" objects
    #============================================#
    def fuse(self, data = {}):
        #TODO in this function
        #1. read data,and judge
        #2. call covariance matrix
        #3. add data with coeff matrix
        #4. break loop, and fuse the added data -> derive integrated data
        #5. return data and covariance matrix
        fused_state = np.zeros((7,1))
        fused_cov = np.zeros((7,7))
        ref_state = np.zeros((7,1))
        #for s_id in data:
        #    print(s_id)

        #print("##############################################")
        for s_id in data:
            data_ = data[s_id]
            rows = len(data_)
            #print("fusing data", s_id)

            for ids in range(rows):
                if data_[ids].status == "detect":
                    #read
                    state = np.zeros((7,1))
                
                    state[0,0] = data_[ids].x
                    state[1,0] = data_[ids].y
                    state[2,0] = data_[ids].r
                    print("ref_r::",data_[ids].ref_r)
                    state[3,0] = data_[ids].theta
                    state[4,0] = data_[ids].w
                    state[5,0] = data_[ids].l
                    state[6,0] = data_[ids].o

                    if s_id == "SVC220.ALL":
                        ref_state[0,0] = data_[ids].ref_x
                        ref_state[1,0] = data_[ids].ref_y
                        ref_state[2,0] = data_[ids].ref_r
                        ref_state[3,0] = data_[ids].ref_theta
                        ref_state[4,0] = data_[ids].ref_w
                        ref_state[5,0] = data_[ids].ref_l
                        ref_state[6,0] = data_[ids].ref_o
                    elif s_id == "ARS510.FC" and ref_state[0,0]==0:
                        #print("ref_setting:", s_id, data_[ids].ref_x)
                        ref_state[0,0] = data_[ids].ref_x
                        ref_state[1,0] = data_[ids].ref_y
                        ref_state[2,0] = data_[ids].ref_r
                        ref_state[3,0] = data_[ids].ref_theta
                        ref_state[4,0] = data_[ids].ref_w
                        ref_state[5,0] = data_[ids].ref_l
                        ref_state[6,0] = data_[ids].ref_o
                    #print("ref setting:", s_id, data_[ids].ref_x)


                    #TODO derive r and theta

                    #mull cov matrix
                    #print("chukan syuturyoku:", state[0,0])
                    s_cov = self.pullcov(s_id, data_[ids])
                    fused_state += inv(s_cov) @ state
                    fused_cov += inv(s_cov)
        print("##############################################")
        
        #fusion observed data
        fused_cov = inv(fused_cov)
        fused_state = fused_cov @ fused_state
        
        obj = objects()
        obj.x = fused_state[0,0]
        obj.y = fused_state[1,0]
        obj.r = fused_state[2,0]
        obj.theta = fused_state[3,0]
        obj.w = fused_state[4,0]
        obj.l = fused_state[5,0]
        obj.o = fused_state[6,0]
        obj.status = "detect"
        obj.cov = fused_cov

        obj.ref_x = ref_state[0,0]
        obj.ref_y = ref_state[1,0]
        obj.ref_r = ref_state[2,0]
        obj.ref_theta = ref_state[3,0]
        obj.ref_w = ref_state[4,0]
        obj.ref_l = ref_state[5,0]
        obj.ref_o = ref_state[6,0]
        #print("fused:", obj.x, obj.ref_x)

        data["fused"] = [obj]

        print("result fused_cov (x_sd, y_sd, w_sd, l_sd, rx_sd, ry_sd):", sqrt(fused_cov[0,0]), sqrt(fused_cov[1,1]), sqrt(fused_cov[2,2]), sqrt(fused_cov[3,3]), sqrt(fused_cov[4,4]), sqrt(fused_cov[5,5]))

        return data

        

    #============================================#
    #function : shift
    #input : data dict
    #output : original data and "fused" objects
    #============================================#   #
    def shift(self, s_id, fusion_point):
        #TODO in this function
        #pass 
        return 0

    def pullcov(self, s_id, obj):
        #TODO in this function
        #vol.1: return general cov matrix
        #vol.2: return calibrated variable matrix changed with 
        #if s_id == "Reference":
        s_id_ = obj.sensor
        std = self.std[s_id_]
        s_cov = np.array([
            [std["x"]**2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, std["y"]**2.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, std["r"]**2.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, std["theta"]**2.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, std["w"]**2.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, std["l"]**2.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, std["o"]**2.0]
        ]) 
        return s_cov



if __name__ == "__main__":
    fusion = HighLevelFuser()
    data = {}
    obj = objects()
    obj.x = 10
    obj.y = 10
    obj.r = 10
    obj.theta = np.pi / 4
    obj.w = 5.0
    obj.l = 1.8
    obj.o = 0.0
    data["ARS510"] = [obj]

    obj = objects()
    obj.x = 15
    obj.y = 15
    obj.r = 21
    obj.theta = np.pi / 4
    obj.w = 6.5
    obj.l = 1.85
    obj.o = 0.0
    data["SVC220"] = [obj]
    fusion.fuse(data, mode = "polar", fusion_point="SVC220")
    print(data["fused"].x)

