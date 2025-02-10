import numpy as np
from numpy import cos, sin, arctan2, pi, sqrt
from numpy.linalg import inv
#from trackers.Struct import objects



#class objects:
#    def __init__(self):
#        self.x = None
#        self.y = None
#        self.v = None
#        self.a = None
#        self.yaw = None
#        self.yawrate = None
#        self.w = None
#        self.l = None


class IMM_EKF:
    def __init__(self):

        #IMM model
        self.s_merge = np.zeros((8,1))
        self.s_cv = np.zeros((8,1))
        self.s_ctrv = np.zeros((8,1))
        self.s_ca = np.zeros((8,1))
        self.s_ctra = np.zeros((8,1))
        self.s_rm = np.zeros((8,1))

        self.p_merge = np.zeros((8,8))
        self.p_cv = np.zeros((8,8))
        self.p_ctrv = np.zeros((8,8))
        self.p_ca = np.zeros((8,8))
        self.p_ctra = np.zeros((8,8))
        self.p_rm = np.zeros((8,8))

        #measument set up
        self.init_timer = None 
        self.timer = None

        #BBF probability
        self.prob = 0.0
        self.L = 0.0

    def init(self, obs, can):
        #TODO in this function
        #1. meas to state
        #2. set up
        if "fused" in obs:
            z = obs["fused"][0]
        else:
            for s_id in obs:
                for obs_id in range(len(obs[s_id])):
                    z = obs[s_id][obs_id]
        state = np.zeros((8,1))
        state[0,0] = z.x
        state[1,0] = z.y
        state[2,0] = sqrt(z.vx**2.0 + z.vy**2.0)
        state[3,0] = 0.0
        state[4,0] = z.o
        state[5,0] = 0.0
        state[6,0] = z.w
        state[7,0] = z.l
        self.s_cv = state
        print("initial state:",state)
        
        self.p_cv = np.identity(8) * 3

        self.init_timer = can[0].timer
        self.timer = can[0].timer

    def predict(self, can, can_pre, p_noise):
        #TODO in this function
        # Relative2Absolute
        # predict
        # Absolute2Relative
        dt = (can[0].timer - can_pre[0].timer) / 10**3#ms -> s
        #=========coordinate transform===========#
        #self.s_cv = self.CoTransformer(self.s_cv, can, can_pre, "absolute")
        #self.s_trcv = self.CoTransformer(self.s_ctrv, can, can_pre, "absolute")
        #self.s_rm = self.CoTransformer(self.s_rm, can, can_pre, "absolute")
        #========================================#

        #===========model prediction=============#
        states = [] #for IMM
        states.append(self.predict_cv(dt))
        #states.append(self.predict_ctrv(dt))
        #states.append(self.predict_rm(dt))
        #========================================#
        
        #=========coordinate transform===========#
        #self.s_cv = self.CoTransformer(self.s_cv, can, can_pre, "relative")
        #self.s_trcv = self.CoTransformer(self.s_ctrv, can, can_pre, "relative")
        #self.s_rm = self.CoTransformer(self.s_rm, can, can_pre, "relative")
        #========================================#

        #============= compute cov ==============#
        covs = [] #for covs
        covs.append(self.predict_p_cv(can, can_pre, p_noise))
        #covs.append(self.predict_p_ctrv(can, can_pre, p_noise))
        #covs.append(self.predict_p_rm(can, can_pre, p_noise))
    
    def update(self, can, can_pre, obs, obs_cov):
        #TODO in this function
        # get measurement noise 
        # compute kalman gain
        # update probability, and state
        dt = (can[0].timer - can_pre[0].timer) / 10**3#ms -> s
        #===========model update=============#
        self.s_cv, self.p_cv = self.update_model(self.s_cv, self.p_cv, obs, obs_cov)
        #self.s_ctrv, self.p_ctrv = self.update_model(self.s_ctrv, self.p_ctrv, obs, obs_cov)
        #self.s_rm, self.p_rm = self.update_model(self.s_rm, self.p_rm, obs, obs_cov)
        states = [self.s_cv, self.s_ctrv, self.s_rm] #for IMM
        covs = [self.p_cv, self.p_ctrv, self.p_rm] #for covs
        #TODO deploy merge multiple model algorithm
        #kokodeha jisyo gata wo amaritukawanai hou gaii kigasuru 
        #========================================#
        self.timer = can[0].timer

    
    def CoTransformer(self, model, can, can_pre, mode="absolute"):

        dt = (can[0].timer - can_pre[0].timer) / 10**3
        v = can[0].v
        v_pre = can_pre[0].v
        a = (v - v_pre) / dt
        yr = can[0].yawrate
        yr_pre = can_pre[0].yawrate
        yaw = (yr - yr_pre) * dt / 2

        if mode == "absolute":
            #model
            s = model
            model[0,0] = sqrt((s[2,0]*cos(s[4,0])+v_pre-s[1,0]*yr_pre)**2.0 + (s[2,0]*sin(s[4,0]) + s[0,0]*yr_pre)**2.0)
            model[1,0] = sqrt((s[3,0]*cos(s[4,0]) + a)**2.0 + (s[3,0]*sin(s[4,0]))**2.0)


        elif mode == "relative":
            #model
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
            model[1,0] = s[1,0] - v_pre * yr_pre * dt**2.0 / 2.0
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
        self.s_cv[0,0] += self.s_cv[2,0] * dt * cos(self.s_cv[4,0]) #x
        self.s_cv[1,0] += self.s_cv[2,0] * dt * sin(self.s_cv[4,0]) #y
        self.s_cv[2,0] += 0.0 #v
        self.s_cv[3,0] += 0.0 #a
        self.s_cv[4,0] += 0.0 #yaw
        self.s_cv[5,0] += 0.0 #yawrate
        self.s_cv[6,0] += 0.0 #w
        self.s_cv[7,0] += 0.0 #l

        print("predicting:", self.s_cv)
        return self.s_cv
    
    #ctrv model
    def predict_ctrv(self, dt):

        v = self.s_ctrv
        self.s_ctrv[0,0] += v[2] / v[5] * ( sin(v[4]+v[5]*dt) - sin(v[4])) #x
        self.s_ctrv[1,0] += v[2] / v[5] * (-cos(v[4]+v[5]*dt) + cos(v[4]))
        self.s_ctrv[2,0] += 0.0
        self.s_ctrv[3,0] += 0.0
        self.s_ctrv[4,0] += v[4] + v[5] * dt
        self.s_ctrv[5,0] += v[5]
        self.s_ctrv[6,0] += 0.0
        self.s_ctrv[7,0] += 0.0

        return self.s_ctrv

    
    #rm model
    def predict_rm(self, dt):
        return self.s_rm


    def predict_p_cv(self, can, can_pre, p_noise):

        #ego car information
        dt = (can[0].timer - can_pre[0].timer) / 10**3
        v = can[0].v
        v_pre = can_pre[0].v
        a = (v - v_pre) / dt
        yr = can[0].yawrate
        yr_pre = can_pre[0].yawrate
        yaw = (yr - yr_pre) * dt / 2
        #target dyanamics information
        t_v = self.s_cv[2,0]
        t_yaw = self.s_cv[4,0]
        dims = self.s_cv.shape[0]

        #derive function
        xpv = dt * cos(t_yaw)
        ypv = dt * sin(t_yaw)
        xpo = -t_v * dt * sin(t_yaw)
        ypo = t_v * dt * cos(t_yaw)

        A = np.array([
                      [1.0, 0.0, xpv, 0.0, xpo, 0.0, 0.0, 0.0],
                      [0.0, 1.0, ypv, 0.0, ypo, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                     ])
        pi = np.array([
                       [ cos(yaw), sin(yaw), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [-sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                      ])
        #CHECK : remove velocity rotate processing
        A = pi @ A
        self.p_cv = A @ A.T @ A + A @ p_noise @ A.T
        return self.p_cv
    
    def predict_p_ctrv(self, can, can_pre, p_noise):

        #ego car information
        dt = (can[0].timer - can_pre[0].timer) / 10**3
        v = can[0].v
        v_pre = can_pre[0].v
        a = (v - v_pre) / dt
        yr = can[0].yawrate
        yr_pre = can_pre[0].yawrate
        yaw = (yr - yr_pre) * dt / 2
        #target dyanamics information
        t_v = self.s_ctrv[2,0]
        t_yaw = self.s_ctrv[4,0]
        dims = self.s_ctrv.shape[0]

        #derive function
        xpv = dt * cos(t_yaw)
        ypv = dt * sin(t_yaw)
        xpo = -t_v * dt * sin(t_yaw)
        ypo = t_v * dt * cos(t_yaw)

        A = np.array([
                      [1.0, 0.0, xpv, 0.0, xpo, 0.0, 0.0, 0.0],
                      [0.0, 1.0, ypv, 0.0, ypo, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                     ])
        pi = np.array([
                       [ cos(yaw), sin(yaw), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [-sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                      ])
        #CHECK : remove velocity rotate processing
        A = pi @ A
        self.p_trcv = A @ A.T @ A + A @ p_noise @ A.T

    def predict_p_rm(self, can, can_pre, p_noise):

        #ego car information
        dt = (can[0].timer - can_pre[0].timer) / 10**3
        v = can[0].v
        v_pre = can_pre[0].v
        a = (v - v_pre) / dt
        yr = can[0].yawrate
        yr_pre = can_pre[0].yawrate
        yaw = (yr - yr_pre) * dt / 2
        #target dyanamics information
        t_v = self.s_rm[2,0]
        t_yaw = self.s_rm[4,0]
        dims = self.s_rm.shape[0]

        #derive function
        xpv = dt * cos(t_yaw)
        ypv = dt * sin(t_yaw)
        xpo = -t_v * dt * sin(t_yaw)
        ypo = t_v * dt * cos(t_yaw)

        A = np.array([
                      [1.0, 0.0, xpv, 0.0, xpo, 0.0, 0.0, 0.0],
                      [0.0, 1.0, ypv, 0.0, ypo, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                     ])
        pi = np.array([
                       [ cos(yaw), sin(yaw), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [-sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                      ])
        #CHECK : remove velocity rotate processing
        A = pi @ A
        self.p_rm = A @ A.T @ A + A @ p_noise @ A.T       

    
    #==========================================#
    def update_model(self, model_, P_,  obs, obs_cov):

        det = {"SVC220.ALL":"SVC220", "ARS510.FC":"ARS510"}
        if "fused" in obs:
            z = obs["fused"][0].state
            R = obs_cov["fused"]
        else:
            for s_id in obs:
                for obs_id in range(len(obs[s_id])):
                    z = obs[s_id][obs_id].state
                    R = obs_cov[det[s_id]]

        dims = P_.shape[0]
        res = z - self.S2M(model_, z)
        H = self.MeasMatrix(model_, z)
        #shape check
        S = R + H @ P_ @ H.T
        K = P_ @ H.T @ inv(S)
        model = model_ + K @ res
        P = (np.identity(dims) - K @ H) @ P_

        return model, P

    #translate state dimentions to meased dimention
    #if reduce measurement vector dimention, modify processing
    def S2M(self, model, z):
        state = np.zeros_like(z)
        state[0,0] = model[0,0]
        state[1,0] = model[1,0]
        state[2,0] = model[2,0] * cos(model[4,0])
        state[3,0] = model[2,0] * sin(model[4,0])
        state[4,0] = model[6,0]
        state[5,0] = model[7,0]
        return state
    
    def MeasMatrix(self, model, z):
        #RADAR and camera matrix are same dimention
        v = model[2,0]
        yaw = model[4,0]
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
    obj = IMM_EKF()

