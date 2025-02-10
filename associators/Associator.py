import numpy as np
from associators.GlobalNearestNeighbor import GNN
#from associators.JPDA import JPDA

class Associator:
    def __init__(self, p_noise, m_noise, mode="GNN"):
        self.associator = None
        self.SelectAssociator(p_noise, m_noise, mode)
        error_message = "Miss initialize tracking objects, checkout target class'" + mode +"' exists"
        assert self.associator != None, error_message

    #==========================================#
    #function : Setect Associator
    #input : 
    #output : 
    #==========================================#
    def SelectAssociator(self, p_noise={}, m_noise={}, mode="GNN"):
        #select model
        #==== GNN =================#
        if mode == "GNN":
            #self.tracker = KF()
            self.associator = GNN(p_noise, m_noise)
        #==== JPDA =================#
        elif mode == "JPDA":
            #self.associator = JPDA()
            pass
        assert self.associator != None, "associator class does not exist......."
                
    #==========================================#
    #function : Compute
    #input : pick up two object list observed data and tracking object data
    #output : index list
    #==========================================#
    def Compute(self, obj_A, obj_B, cov_A, cov_B, mode="direct"):
        #TODO in this function
        result = self.associator.compute(obj_A, obj_B, cov_A, cov_B, mode)
        return result
