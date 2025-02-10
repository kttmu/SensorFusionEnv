import numpy as np
from associators.munker import Munkres
#from ..config.NoiseLoader import NoiseLoader


##=========================================
## Classification of 3 pattern (Pair, New, Lost)
##=========================================
class GNN:
    def __init__(self, p_noise, m_noise):
        self.m = Munkres()
        self.chisq = 18.6
        #self.Loader = NoiseLoader()
        self.m_noise = m_noise
        self.p_noise = p_noise
        #self.init_covariance(args)

    #def init_covariance(self):
    #    #measurement noise phase
    #    file = "./config/m_noise_cov.npz"
    #    covs = self.Loader.load(file)
    #    for s_id in covs.files:
    #        self.m_noise[s_id] = covs[s_id]
    #    #processing noise phase
    #    file = "./config/p_noise_cov.npy"
    #    self.p_noise = self.Loader.load(file)
        
    #def Global_Nearest_Neighbor(self, obj_A, obj_B):
    def compute(self, obj_A, obj_B, covA, covB, mode="run"):
    
        ## Create Cost-Matrix[l x m]
        l = len(obj_A)
        m = len(obj_B)
        C = np.zeros((l, m))
    
        for i in range(l):
            for j in range(m):
    
                ## Calc. Covariance
                #derive state
                z_A = np.array([
                    obj_A[i].x,
                    obj_A[i].y,
                    obj_A[i].vx,
                    obj_A[i].vy,
                    obj_A[i].w,
                    obj_A[i].l,
                ])
                if mode=="inverse":
                    z_B = np.array([
                        obj_B[j].x,
                        obj_B[j].y,
                        obj_A[i].vx,
                        obj_A[i].vy,
                        obj_B[j].w,
                        obj_B[j].l,
                    ])
                    #covA = covA * 2.0
                    #covB = covB * 2.0
                    #print("run clac")
                else:
                    z_B = np.array([
                        obj_B[j].x,
                        obj_B[j].y,
                        obj_B[j].vx,
                        obj_B[j].vy,
                        obj_B[j].w,
                        obj_B[j].l,
                    ])
                #print("mode:",mode)
                #print("A:", z_A)
                #print("B:", z_B)
                #z_B = obj_B[j]
                #derive covariance
                #obj_A_cov = self.p_noise
                #obj_B_cov = self.p_noise
                obj_A_cov = covA
                obj_B_cov = covB
                
                
                z_diff = z_A - z_B
                #print("Acov:", obj_A_cov)
                #print("Bcov:", obj_B_cov)
                
                S = obj_A_cov + obj_B_cov
                #print("Shape of z and S:", z_diff.shape, S.shape)
                d = z_diff.T @ np.linalg.inv(S) @ z_diff
                ## Chi-square test
                #print(d)
                if d < self.chisq:
                    C[i,j] = d
                else:
                    C[i,j] = 99999999 #np.inf
    
        #Cshape
        C = self.Expand_matrix(C)
        #print("cost matrix shape:", C)
        #C expand shape
        result = self.m.compute(C)
        result = self.postprocessing(obj_A, obj_B, result)
    
        return result
    
    def Expand_matrix(self, C):
        l, m = C.shape[0], C.shape[1]

        up_right_mat = np.full((l,l), 99999999) - np.eye(l)*10000
        down_left_mat = np.full((m,m), 99999999) - np.eye(m)*10000
        down_right_mat = np.full((m,l), 99999999)

        C2 = np.concatenate([C, up_right_mat], axis=1)
        down_mat = np.concatenate([down_left_mat, down_right_mat], axis=1)
        C3 = np.concatenate([C2, down_mat], axis=0)

        return C3

    #extracting matched pairs
    def postprocessing(self, obj_A, obj_B, result):
        l = len(obj_A)
        m = len(obj_B)
        result_ = []

        for row, col in result:
            if row < l and col < m:
                new = (row, col)
                result_.append(new)
        return result_


if __name__=="__main__":
    A = []
    B = []
    A_cov = np.identity(4)*3.0
    B_cov = np.identity(4)*3.0
    a = np.array([[1],[1.5],[1.5],[2]])
    b = np.array([[1.5],[1.2],[1.1],[1.8]])
    A = [a, a*2, a*1.5, a*1.8]
    B = [b, b*2, b*1.5, b*1.8]
    gnn = GlobalNearestNeighbor()
    result = gnn.compute(A, B, A_cov, B_cov)
    print(result)