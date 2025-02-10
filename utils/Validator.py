import numpy as np
#from common.Struct import objects
#from Struct import objects

class Result:
    def __init__(self):
        self.x = None
        self.y = None
        self.r = None
        self.theta = None
        self.w = None
        self.l = None
        self.o = None
        self.rx = None
        self.ry = None

class Validator:
    def __init__(self, target):
        self.res_x = []
        self.res_y = []
        self.res_r = []
        self.res_theta = []
        self.res_rx = []
        self.res_ry = []
        self.res_w = []
        self.res_l = []
        self.res_o = []

        self.res_xx = []
        self.res_yy = []
        self.res_rr = []
        self.res_tt = []
        self.res_ww = []
        self.res_ll = []
        self.res_oo = []
        self.res_rxrx = []
        self.res_ryry = []

        self.target = target

    #============================================#
    #function : accumulate
    #input : data dict
    #output : nothing
    #============================================#
    def accumulate(self, data = {}):
        s_id = self.target
        data_ = data[s_id]
        print("accumu",s_id)
        rows = len(data_)
        for ids in range(rows):
            if data_[ids].status == "detect":
                print(data_[ids])

                #accumulate residual data
                self.res_xx.append((data_[ids].x - data_[ids].ref_x)**2)
                self.res_yy.append((data_[ids].y - data_[ids].ref_y)**2)
                self.res_ww.append((data_[ids].w - data_[ids].ref_w)**2)
                self.res_ll.append((data_[ids].l - data_[ids].ref_l)**2)
                self.res_oo.append((data_[ids].o - data_[ids].ref_o)**2)
                self.res_tt.append((data_[ids].theta - data_[ids].ref_theta)**2)
                self.res_rr.append((data_[ids].r - data_[ids].ref_r)**2)

                self.res_x.append(data_[ids].x - data_[ids].ref_x)
                self.res_y.append(data_[ids].y - data_[ids].ref_y)
                self.res_w.append(data_[ids].w - data_[ids].ref_w)
                self.res_l.append(data_[ids].l - data_[ids].ref_l)
                self.res_o.append(data_[ids].o - data_[ids].ref_o)
                self.res_r.append(data_[ids].r - data_[ids].ref_r)
                self.res_theta.append(data_[ids].theta - data_[ids].ref_theta)

                #derie rxrx 
                rxrx = data_[ids].r * np.cos(data_[ids].theta)
                ryry = data_[ids].r * np.sin(data_[ids].theta)
                print("################")
                print("check rx::", rxrx, data_[ids].ref_x, ryry, data_[ids].ref_y)
                print("check r::", data_[ids].r, data_[ids].ref_r)
                print("################")
                self.res_rx.append(rxrx - data_[ids].ref_x)
                self.res_ry.append(ryry - data_[ids].ref_y)
                self.res_rxrx.append((rxrx - data_[ids].ref_x)**2.0)
                self.res_ryry.append((ryry - data_[ids].ref_y)**2.0)
                #self.res_r.append(data_[ids].r - data_[ids].ref_r)
                #self.res_theta.append(data_[ids].theta - data_[ids].ref_theta)

     
    #============================================#
    #function : validate
    #input : data dict
    #output : an dict that has same shape of input data
    #============================================#
    def validate(self):
       
        #translate
        len_x = len(self.res_x)
        len_y = len(self.res_y)
        len_w = len(self.res_w)
        len_l = len(self.res_l)
        len_o = len(self.res_o)

        self.res_x = np.array(self.res_x)
        self.res_y = np.array(self.res_y)
        self.res_r = np.array(self.res_r)
        self.res_theta = np.array(self.res_theta)
        self.res_w = np.array(self.res_w)
        self.res_l = np.array(self.res_l)
        self.res_o = np.array(self.res_o)
        self.res_rx = np.array(self.res_rx)
        self.res_ry = np.array(self.res_ry)

        self.res_xx = np.array(self.res_xx)
        self.res_yy = np.array(self.res_yy)
        self.res_rr = np.array(self.res_rr)
        self.res_tt = np.array(self.res_tt)
        self.res_ww = np.array(self.res_ww)
        self.res_ll = np.array(self.res_ll)
        self.res_oo = np.array(self.res_oo)
        self.res_rxrx = np.array(self.res_rxrx)
        self.res_ryry = np.array(self.res_ryry)
        print(self.target)
        print(self.res_x)

        #derive std
        SD = Result()
        SD.x = np.sqrt(np.sum(self.res_xx) / len_x)
        SD.y = np.sqrt(np.sum(self.res_yy) / len_y)
        SD.w = np.sqrt(np.sum(self.res_ww) / len_w)
        SD.l = np.sqrt(np.sum(self.res_ll) / len_l)
        SD.o = np.sqrt(np.sum(self.res_oo) / len_o)
        SD.r = np.sqrt(np.sum(self.res_rr) / len_o)
        SD.theta = np.sqrt(np.sum(self.res_tt) / len_o)
        SD.rx = np.sqrt(np.sum(self.res_rxrx) / len_o)
        SD.ry = np.sqrt(np.sum(self.res_ryry) / len_o)

        #SD.r = np.std(self.res_r)
        #SD.theta = np.std(self.res_theta)

        #derive avg
        AVG = Result()
        AVG.x = np.mean(self.res_x)
        AVG.y = np.mean(self.res_y)
        #AVG.r = np.mean(self.res_r)
        #AVG.theta = np.mean(self.res_theta)
        AVG.w = np.mean(self.res_w)
        AVG.l = np.mean(self.res_l)
        AVG.o = np.mean(self.res_o)

        return SD, AVG
