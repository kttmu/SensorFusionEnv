import numpy as np
from numpy import cos, sin, arctan2, pi, sqrt
from numpy.linalg import inv
#from trackers.Struct import objects


#state dimention
"""
0:x
1:y
2:vx
3:vy
4:ax
5:ay
6:w
7:l

warning:
cov matrix must be received ndarray obejct.
"""

class IMM_KF:
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
        self.data = {"SVC220":[], "ARS510":[], "SRR520_FL":[], "SRR520_FR":[], "SRR520_RL":[], "SRR520_RR":[], "fused":[]}
        #self.p_noise = np.array([[1.0,0.0,0.0,0.0,00.0,0.0,0.0,0.0],
        #                         [0.0,1.0,0.0,dt,0.0,0.5*dt**2.0,0.0,0.0],
        #                         [0.0,0.0,1.0,0.0,dt,0.0,0.0,0.0],
        #                         [0.0,0.0,0.0,1.0,0.0,dt,0.0,0.0],
        #                         [0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0],
        #                         [0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0],
        #                         [0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0],
        #                         [0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0]
        #                       ])
        self.p_noise = np.identity(8) * 2
        #copy to element
        self.x  = 0.0
        self.y  = 0.0 
        self.vx = 0.0 
        self.vy = 0.0 
        self.ax = 0.0 
        self.ay = 0.0 
        self.w  = 0.0 
        self.l  = 0.0 
        self.o  = 0.0 

        

    def init(self, obs, can):
        #TODO in this function
        #1. meas to state
        #2. set up
        if len(obs["fused"]) > 0:
            z = obs["fused"][0]
        else:
            for s_id in obs:
                for obs_id in range(len(obs[s_id])):
                    z = obs[s_id][obs_id]
        state = np.zeros((9,1))
        state[0,0] = z.x
        state[1,0] = z.y
        state[2,0] = z.vx
        state[3,0] = z.vy
        state[4,0] = 0.0
        state[5,0] = 0.0
        state[6,0] = z.w
        state[7,0] = z.l
        self.s_cv = state
        self.s_ca = state[0:8]
        
        self.p_cv = np.identity(8) * 3
        self.p_ca = np.identity(8) * 3

        self.init_timer = can[0].timer
        self.timer = can[0].timer
        
        #copy to element
        self.x =  self.s_ca[0,0] 
        self.y =  self.s_ca[1,0] 
        self.vx = self.s_ca[2,0] 
        self.vy = self.s_ca[3,0] 
        self.ax = self.s_ca[4,0] 
        self.ay = self.s_ca[5,0] 
        self.w =  self.s_ca[6,0] 
        self.l =  self.s_ca[7,0]

    def predict(self, can, can_pre, p_noise):
        #TODO in this function
        # Relative2Absolute
        # predict
        # Absolute2Relative
        dt = (can[0].timer - can_pre[0].timer) / 10**3#ms -> s
        #=========coordinate transform===========#
        self.s_ca = self.CoTransformer(self.s_ca, can, can_pre, "absolute")
        #self.s_trcv = self.CoTransformer(self.s_ctrv, can, can_pre, "absolute")
        #self.s_rm = self.CoTransformer(self.s_rm, can, can_pre, "absolute")
        #========================================#

        #===========model prediction=============#
        states = [] #for IMM
        states.append(self.predict_ca(dt))
        #states.append(self.predict_ctrv(dt))
        #states.append(self.predict_rm(dt))
        #========================================#
        
        #=========coordinate transform===========#
        self.s_ca = self.CoTransformer(self.s_ca, can, can_pre, "relative")
        #self.s_cv = self.CoTransformer(self.s_cv, can, can_pre, "relative")
        #self.s_trcv = self.CoTransformer(self.s_ctrv, can, can_pre, "relative")
        #self.s_rm = self.CoTransformer(self.s_rm, can, can_pre, "relative")
        #========================================#

        #============= compute cov ==============#
        covs = [] #for covs
        covs.append(self.predict_p_ca(can, can_pre, p_noise))
        #covs.append(self.predict_p_ctrv(can, can_pre, p_noise))
        #covs.append(self.predict_p_rm(can, can_pre, p_noise))
         
        #copy to element
        self.x =  self.s_ca[0,0] 
        self.y =  self.s_ca[1,0] 
        self.vx = self.s_ca[2,0] 
        self.vy = self.s_ca[3,0] 
        self.ax = self.s_ca[4,0] 
        self.ay = self.s_ca[5,0] 
        self.w =  self.s_ca[6,0] 
        self.l =  self.s_ca[7,0]
    
    def update(self, can, can_pre, obs, obs_cov):
        #TODO in this function
        # get measurement noise 
        # compute kalman gain
        # update probability, and state
        dt = (can[0].timer - can_pre[0].timer) / 10**3#ms -> s
        #===========model update=============#
        #self.s_cv, self.p_cv = self.update_model(self.s_cv, self.p_cv, obs, obs_cov)
        self.s_ca, self.p_ca = self.update_model(self.s_ca, self.p_ca, obs, obs_cov)
        #self.s_ctrv, self.p_ctrv = self.update_model(self.s_ctrv, self.p_ctrv, obs, obs_cov)
        #self.s_rm, self.p_rm = self.update_model(self.s_rm, self.p_rm, obs, obs_cov)
        states = [self.s_cv, self.s_ctrv, self.s_rm] #for IMM
        covs = [self.p_cv, self.p_ctrv, self.p_rm] #for covs
        #TODO deploy merge multiple model algorithm
        #kokodeha jisyo gata wo amaritukawanai hou gaii kigasuru 
        #========================================#
        self.timer = can[0].timer
        
        #copy to element
        self.x =  self.s_ca[0,0] 
        self.y =  self.s_ca[1,0] 
        self.vx = self.s_ca[2,0] 
        self.vy = self.s_ca[3,0] 
        self.ax = self.s_ca[4,0] 
        self.ay = self.s_ca[5,0] 
        self.w =  self.s_ca[6,0] 
        self.l =  self.s_ca[7,0]

    
    def CoTransformer(self, model, can, can_pre, mode="absolute"):

        dt = (can[0].timer - can_pre[0].timer) / 10**3
        if dt == 0:
            dt = 0.0000000001
        v = can[0].v
        v_pre = can_pre[0].v
        a = (v - v_pre) / dt
        yr = np.deg2rad(can[0].yawrate)
        yr_pre = np.deg2rad(can_pre[0].yawrate)
        yaw = (yr - yr_pre) * dt * 0.5

        if mode == "absolute":
            #mass product(8)~(9)
            model[2,0] = model[2,0] - yr_pre * model[1,0] + v_pre
            model[3,0] = model[3,0] + yr_pre * model[0,0]
            model[4,0] = model[4,0] + a
            model[5,0] = model[5,0] + yr_pre * v_pre

        elif mode == "relative":
            #model
            pi = np.array([[ cos(yaw), sin(yaw), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                           [ -sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, cos(yaw), sin(yaw),  0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, -sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, cos(yaw), sin(yaw), 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, -sin(yaw), cos(yaw),0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

            #absolute 2 relative transform
            model[0,0] = model[0,0] - v_pre * dt - a * dt**2.0 * 0.5
            model[1,0] = model[1,0] - v_pre * yr_pre * dt**2.0 * 0.5
            shift = np.array([[0.0],
                              [0.0],
                              [-yr * model[1,0] + v*cos(yaw)], 
                              [yr * model[0,0] + v*sin(yaw)], 
                              [-v*yr*sin(yaw) + a], 
                              [v*yr*cos(yaw)], 
                              [0.0], 
                              [0.0]])

            model -= shift
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

        return self.s_cv
    
    def predict_ca(self, dt):
        #cv model predict
        A = np.array([[1.0,0.0,dt,0.0,0.5*dt**2.0,0.0,0.0,0.0],
                      [0.0,1.0,0.0,dt,0.0,0.5*dt**2.0,0.0,0.0],
                      [0.0,0.0,1.0,0.0,dt,0.0,0.0,0.0],
                      [0.0,0.0,0.0,1.0,0.0,dt,0.0,0.0],
                      [0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0],
                      [0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0],
                      [0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0],
                      [0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0]])
        self.s_ca = A @ self.s_ca

        return self.s_ca
    
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

    def predict_p_ca(self, can, can_pre, p_noise):

        #ego car information
        dt = (can[0].timer - can_pre[0].timer) / 10**3
        if dt == 0:
            dt = 0.0000000001
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

        A0 = np.array([
                      [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, -yr_pre, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [yr_pre, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                     ])
        A1 = np.array([
                      [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, yr, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [-yr, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                     ])
       
        G = np.array([[1.0,0.0,dt,0.0,0.5*dt**2.0,0.0,0.0,0.0],
                          [0.0,1.0,0.0,dt,0.0,0.5*dt**2.0,0.0,0.0],
                          [0.0,0.0,1.0,0.0,dt,0.0,0.0,0.0],
                          [0.0,0.0,0.0,1.0,0.0,dt,0.0,0.0],
                          [0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0],
                          [0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0],
                          [0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0],
                          [0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0]
                        ])
        pi = np.array([[ cos(yaw), sin(yaw), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [ -sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, cos(yaw), sin(yaw),  0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, -sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, cos(yaw), sin(yaw), 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, -sin(yaw), cos(yaw),0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
        
        #integrate
        Q = G @ self.p_noise @ G.T
        A = pi @ A1 @ G @ A0
        self.p_ca = A @ self.p_ca @ A.T + Q

         
        return self.p_ca

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
        G = np.array([[1.0,0.0,dt,0.0,0.5*dt**2.0,0.0,0.0,0.0],
                      [0.0,1.0,0.0,dt,0.0,0.5*dt**2.0,0.0,0.0],
                      [0.0,0.0,1.0,0.0,dt,0.0,0.0,0.0],
                      [0.0,0.0,0.0,1.0,0.0,dt,0.0,0.0],
                      [0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0],
                      [0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0],
                      [0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0],
                      [0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0]
                    ])
        #CHECK : remove velocity rotate processing
        A = pi @ A
        #self.p_cv = A @ A.T @ A + A @ p_noise @ A.T
        self.p_cv = A @ self.p_cv @ A.T + G @ self.p_noise @ G.T
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

        if len(obs["fused"]) > 0:
            z = np.zeros((6,1))
            z[0,0] = obs["fused"][0].x
            z[1,0] = obs["fused"][0].y
            z[2,0] = obs["fused"][0].vx
            z[3,0] = obs["fused"][0].vy
            z[4,0] = obs["fused"][0].w
            z[5,0] = obs["fused"][0].l
            R = obs_cov["fused"]
        else:
            for s_id in obs:
                for obs_id in range(len(obs[s_id])):
                    #z = obs[s_id][obs_id].state
                    z = np.zeros((6,1))
                    z[0,0] = obs[s_id][obs_id].x
                    z[1,0] = obs[s_id][obs_id].y
                    z[2,0] = obs[s_id][obs_id].vx
                    z[3,0] = obs[s_id][obs_id].vy
                    z[4,0] = obs[s_id][obs_id].w
                    z[5,0] = obs[s_id][obs_id].l
                    R = obs_cov[s_id]

        H = self.MeasMatrix(model_, z)
        res = z - H @ model_
        #shape check
        S = R + H @ P_ @ H.T
        K = P_ @ H.T @ inv(S)
        model = model_ + K @ res
        P = (np.identity(8) - K @ H ) @ P_
        #print("update: ", model)
        #print(self.prob)

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
                      [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                      ])
        return H





if __name__=="__main__":
    #EKF test
    obj = IMM_KF()

