import numpy as np
# =================== Obsevated object list
class Observed_Object:
    def __init__(self):
        self.y = np.zeros(7)
        # [0]: x
        # [1]: y
        # [2]: Vx
        # [3]: Vy
        # [4]: width
        # [5]: length
        # [6]: orientation
        self.cov = np.zeros((7,7))

        self.ui_time_stamp = 0
        self.sensor_src = 0
        self.e_classification = 0
        self.f_probability_of_existence = 0
        self.ui_life_cycles = 0

class Observed_Object_List:
    def __init__(self):
        self.time = 0
        self.obj = []

# =================== Tracking object list
class Tracking_Object:
    def __init__(self):
        self.ID = 0
        self.age = 0
        self.unobserved_count = 0
        self.x_p = np.zeros(9)
        self.x_s = np.zeros(9)
        # [0]: x
        # [1]: y
        # [2]: Vx
        # [3]: Vy
        # [4]: ax
        # [5]: ay
        # [6]: width
        # [7]: length
        # [8]: orientation

        self.P_p = np.zeros((9,9))
        self.P_s = np.zeros((9,9))
        self.data = {"SVC220":[], "ARS510":[], "SRR520_FL":[], "SRR520_FR":[], "SRR520_RL":[], "SRR520_RR":[]}

class Tracking_Object_List:
    def __init__(self):
        self.time = 0
        self.obj = []
        self.ID_list = []