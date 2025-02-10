import numpy as np
from associators.munker import Munkres
#from ..config.NoiseLoader import NoiseLoader


##=========================================
## Classification of 3 pattern (Pair, New, Lost)
##=========================================
class GNN:
    def __init__(self, p_noise, m_noise):
        self.m = Munkres()
        self.chisq = 18.5
        #self.Loader = NoiseLoader()
        self.m_noise = m_noise
        self.p_noise = p_noise
        #self.init_covariance(args)

    def init_covariance(self):
        #measurement noise phase
        file = "./config/m_noise_cov.npz"
        covs = self.Loader.load(file)
        for s_id in covs.files:
            self.m_noise[s_id] = covs[s_id]
        #processing noise phase
        file = "./config/p_noise_cov.npy"
        self.p_noise = self.Loader.load(file)
        
    #def Global_Nearest_Neighbor(self, obj_A, obj_B):
    """データ構造を同設計するべきだろうか⇒奥さんの丸パクッテいいや"""
    def compute(self, obj_A, obj_B):
    
        ## Create Cost-Matrix[l x m]
        l = len(obj_A)
        m = len(obj_B)
        C = np.zeros((l, m))
    
        for i in range(l):
            for j in range(m):
    
                ## Calc. Covariance
                z_A = obj_A[i]
                z_B = obj_B[j]
                print("vecA:", z_A[i].x)
                print("vecB:", z_B[i].x)
                z_A = np.array([[obj_A[i].x],
                                [obj_A[i].y],
                                [obj_A[i].vx],
                                [obj_A[i].vx],
                                [obj_A[i].w],
                                [obj_A[i].l]
                ])
                z_B = np.array([[obj_B[i].x],
                                [obj_B[i].y],
                                [obj_B[i].vx],
                                [obj_B[i].vx],
                                [obj_B[i].w],
                                [obj_B[i].l]
                ])
                #derive covariance
                obj_A_cov = self.p_noise["track"]
                obj_B_cov = self.p_noise["obs"]
                
                
                z_diff = z_A - z_B
                S = obj_A_cov + obj_B_cov
                d = z_diff.T @ np.linalg.inv(S) @ z_diff
                ## Chi-square test
                if d < self.chisq:
                    C[i,j] = d
                else:
                    C[i,j] = 99999999 #np.inf
    
        #Cshape
        print(C.shape)
        C = self.Expand_matrix(C)
        #C expand shape
        print(C.shape)
        result = self.m.compute(C)
    
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