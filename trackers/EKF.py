import numpy as np
from numpy import cos, sin, arctan2, pi, sqrt
from numpy.linalg import inv
import trackers.Struct import objects



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


class EKF:
    def __init__(self):

        #IMM model
        self.s_merge = np.zeros((8,1))
        self.s_cv = np.zeros((8,1))
        self.s_ctrv = np.zeros((8,1))
        self.s_ca = np.zeros((8,1))
        self.s_ctra = np.zeros((8,1))
        self.s_rm = np.zeros((8,1))

        self.p_meerge = np.zeros((8,8))
        self.p_cv = np.zeros((8,8))
        self.p_ctrv = np.zeros((8,8))
        self.p_ca = np.zeros((8,8))
        self.p_ctra = np.zeros((8,8))
        self.p_rm = np.zeros((8,8))

        #measument set up


    def predict(self, can, can_pre):
        #TODO in this function
        # Relative2Abs
        # predict
        # ABS2Relative
        dt = (can - can_pre) * 10**3.0#ms -> s
        #=========coordinate transform===========#
        self.s_cv = self.CoTransformer(self.s_cv, can, can_pre, "absolute")
        self.s_trcv = self.CoTransformer(self.s_ctrv, can, can_pre, "absolute")
        self.s_rm = self.CoTransformer(self.s_rm, can, can_pre, "absolute")
        #========================================#

        #===========model prediction=============#
        states = [] #for IMM
        states.append(self.predict_cv(dt))
        states.append(self.predict_ctrv(dt))
        states.append(self.predict_rm(dt))
        #========================================#
        
        #=========coordinate transform===========#
        self.CoTransformer(can, can_pre, "relative")
        self.s_cv = self.CoTransformer(self.s_cv, can, can_pre, "relative")
        self.s_trcv = self.CoTransformer(self.s_ctrv, can, can_pre, "relative")
        self.s_rm = self.CoTransformer(self.s_rm, can, can_pre, "relative")
        #========================================#

        #============= compute cov ==============#
        covs = [] #for covs
        covs.append(self.predict_cov_cv(dt))
        covs.append(self.predict_cov_ctrv(dt))
        covs.append(self.predict_cov_rm(dt))
    
    def update(self, can, can_pre, obs, obs_cov):
        #TODO in this function
        # get measurement noise 
        # compute kalman gain
        # update probability, and state
        dt = (can - can_pre) * 10**3.0#ms -> s
        #===========model update=============#
        self.s_cv, self.P_cv = self.update_model(self.s_cv, self.P_cv, obs, obs_cov)
        self.s_ctrv, self.P_ctrv = self.update_model(self.s_ctrv, self.P_ctrv, obs, obs_cov)
        self.s_rm, self.P_rm = self.update_model(self.s_rm, self.P_rm, obs, obs_cov)
        states = [self.s_cv, self.s_ctrv, self.s_rm] #for IMM
        covs = [self.P_cv, self.P_ctrv, self.P_rm] #for covs
        #kokodeha jisyo gata wo amaritukawanai hou gaii kigasuru 
        #========================================#

    
    def CoTransformer(self, model, can, can_pre, mode="absolute"):

        dt = (can[0].timer - can[0].timer) * 10**3
        v = can[0].v
        v_pre = can_pre[0].v
        a = (v - v_pre) / dt
        yr = can[0].yawrate
        yr_pre = can_pre[0].yawrate
        yaw = (yr - yr_pre) * dt / 2

        if mode == "absolute":

            s = model
            model[0,0] = sqrt((s[2,0]*cos(s[4,0])+v_pre-s[1,0]*yr_pre)**2.0 + (s[2,0]*sin(s[4,0]) + s[0,0]*yr_pre)**2.0)
            model[1,0] = sqrt((s[3,0]*cos(s[4,0]) + a)**2.0 + (s[3,0]*sin(s[4,0]))**2.0)

        elif mode == "relative":

            pi = np.array([[ cos(yaw), sin(yaw), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                           [-sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
            s = model
            model[0,0] = s[0,0] - v_pre * dt - a * dt**2.0 / 2.0
            model[1,0] = s[0,1] - v_pre * yr_pre * dt**2.0 / 2.0
            model[2,0] = sqrt((-s[1,0]*yr + v_pre * cos(yaw))**2.0 + (s[0,0] * yr + v * sin(yr))**2.0)
            model[3,0] = sqrt((a - v * yr_pre * sin(yaw))**2.0 + (v * yr_pre * cos(yaw))**2.0) 
            model[4,0] = model[4,0]
            model[5,0] = model[5,0]
            model[6,0] = model[6,0]
            model[7,0] = model[7,0]
            model = pi @ model

        return model




    #========== prediction module ==============#
    #cv model
    def predict_cv(self, dt):
        #cv model predict
        self.s_cv[0,0] += self.s_cv[2] * dt * cos(self.s_cv[4]) #x
        self.s_cv[1,0] += self.s_cv[2] * dt * sin(self.s_cv[4]) #y
        self.s_cv[2,0] += 0.0 #v
        self.s_cv[3,0] += 0.0 #a
        self.s_cv[4,0] += 0.0 #yaw
        self.s_cv[5,0] += 0.0 #yawrate
        self.s_cv[6,0] += 0.0 #w
        self.s_cv[7,0] += 0.0 #l

        return self.vec_cv
    
    #ctrv model
    def predict_ctrv(self, dt):

        v = self.vec_ctrv
        self.s_ctrv[0,0] += v[2] / v[5] * ( sin(v[4]+v[5]*dt) - sin(v[4])) #x
        self.s_ctrv[1,0] += v[2] / v[5] * (-cos(v[4]+v[5]*dt) + cos(v[4]))
        self.s_ctrv[2,0] += 0.0
        self.s_ctrv[3,0] += 0.0
        self.s_ctrv[4,0] += v[4] + v[5] * dt
        self.s_ctrv[5,0] += v[5]
        self.s_ctrv[6,0] += 0.0
        self.s_ctrv[7,0] += 0.0

        return self.vec_ctrv

    
    #rm model
    def predict_rm(self, dt):
        return self.s_rm
    
    #==========================================#
    def update_model(self, model_, P_,  obs, obs_cov):

        if "fused" in obs:
            z = obs["fused"][0].state
            R = obs_cov["fused"]
        for s_id in obs:
            z = obs[s_id][0].state
            R = obs_cov[s_id]

        dims = P_.shape[0]
        res = z - self.S2M(model_, z)
        H = self.MeasMatrix(model_, z)
        S = R + H @ P_ @ H.T
        K = P_ @ H.T @ inv(S)
        model = model_ + K @ res
        P = (np.identity(dims) - K @ H) @ P_

        return model, P

    def S2M(self, model, z):
        state = np.zeros_like(z)
        state[0,0] = model[0,0]
        state[1,0] = model[1,0]
        state[2,0] = model[2,0] * cos(model[4,0])
        state[3,0] = model[0,0] * sin(model[4,0])
        state[4,0] = model[6,0]
        state[5,0] = model[7,0]
        return state
    
    def MeasMatrix(self, model, z):
        #RADAR and camera matrix are same dimention
        v = model[2]
        yaw = model[4]
        H = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, cos(yaw), 0.0, -v*sin(yaw), 0.0, 0.0, 0.0],
                      [0.0, 0.0, sin(yaw), 0.0,  v*cos(yaw), 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                      ])
        return H





if __name__=="__main__":
    #EKF test
    obj = EKF()

