#class import
import numpy as np
import copy
from DataLoader import Loader
from trackers.Tracker import Tracker
from associators.Associator import Associator
from associators.HilevelFuser import HighLevelFuser
from associators.BinaryBayesFilter import BBF

#config import
from config.NoiseLoader import NoiseLoader
from config.option_parser import option_parser
#data structure import
from associators.structure_tracking import Tracking_Object_List, Observed_Object_List, Tracking_Object, Observed_Object
#logger
import csv
#visualizer
from utils.PlotFigure import PlotFigure

z_min = -100
z_max = 100
#p_exist = 0.6
#p_hold = 0.45
p_exist = 0.7
p_hold = 0.3
#main processing flow
class Manager:
    def __init__(self, args, p_noise=None):
        #Set measurement Noise and process Noise
        Noise = NoiseLoader()
        #self.sensors, self.m_noise = Noise.load("./config/m_noise_cov.npz")
        _, self.p_noise = Noise.load("./config/p_noise_cov.npy")
        self.PlotFigure = PlotFigure(args, Video=1)
        #_, self.p_noise = Noise.load("./config/P_noise_cov.npy")
        #TBD TODO later..: set measurement noise to covariance noise
        self.m_noise = {}
        _, self.m_noise["ARS510"]    = Noise.load("./config/m_noise_cov.npy")
        _, self.m_noise["SVC220"]    = Noise.load("./config/m_noise_cov.npy")
        _, self.m_noise["SRR520_FR"] = Noise.load("./config/m_noise_cov.npy")
        _, self.m_noise["SRR520_FL"] = Noise.load("./config/m_noise_cov.npy")
        _, self.m_noise["SRR520_RR"] = Noise.load("./config/m_noise_cov.npy")
        _, self.m_noise["SRR520_RL"] = Noise.load("./config/m_noise_cov.npy")
        _, self.m_noise["fused"]     = Noise.load("./config/m_noise_cov.npy")

        _, self.m_noise["ARS510"]    = Noise.load("./config/ars510_cov.npy")
        _, self.m_noise["SVC220"]    = Noise.load("./config/svc220_cov.npy")
        _, self.m_noise["SRR520_FR"] = Noise.load("./config/srr520_cov.npy")
        _, self.m_noise["SRR520_FL"] = Noise.load("./config/srr520_cov.npy")
        _, self.m_noise["SRR520_RR"] = Noise.load("./config/srr520_cov.npy")
        _, self.m_noise["SRR520_RL"] = Noise.load("./config/srr520_cov.npy")

        _, self.m_noise["fused"]     = Noise.load("./config/ars510_cov.npy")
        #print(self.m_noise["SRR520_RL"])
        #s = input()
        for s_id in self.m_noise:
            self.m_noise[s_id] *= 1
        self.p_noise *= 1.5

        #Set data loader and read initial measurement points
        self.Data = Loader(args)
        #self.img, self.data, self.can, self.flg, self.timer = self.Data.read()
        self.img, self.data, self.can, self.flg, self.timer, lidar = self.Data.read()
        self.data_  = self.data

        #Set config (tracking mode, association mode, )
        self.track_mode = args.tracker
        self.Associator = Associator(self.p_noise, self.m_noise, mode=args.associator)

        #call private variables
        self.can_pre = None
        self.TrackingObjList = []

        #import filtering processing module
        self.BBF = BBF()
        self.Fuser = HighLevelFuser()
        self.cycle_counter = -1

        #TODO inclue measumenr matrix from other class
        self.H = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                    ])
        self.args = args
        if self.args.evaluator:
            #with open("../raw_data/" + self.args.data_name + "result_tracker=" + self.args.tracker + "_asso="+self.args.associator+"_xtss="+self.args.xtss+".csv", mode="w", newline="")as f:
            with open("../raw_data/" + self.args.data_name + "result_tracker=" + self.args.tracker + "_asso="+self.args.associator+"_xtss="+self.args.xtss+"_mode="+str(self.args.tuning)+".csv", mode="w", newline="")as f:
                writer = csv.writer(f)
                writer.writerow(["timer", "tar_x", "tar_y", "tar_w", "tar_l", "tar_theta", "tar_class", "status", "sensor", "obs_x", "obs_y", "obs_w", "obs_l", "obs_theta", "obs_class", "obs_prob"])
        
        #optimization flag
        self.tuning_mode = args.tuning
        if self.tuning_mode:
            self.p_noise = p_noise


    ##################################################
    #function : run
    #input : nothing
    #output : nothing(tuning mode:cost_, residual)
    #abstruct : Filtering and management all tracking objects with added observed data
    ##################################################
    def run(self):

        #Initialize
        self.cycle_counter += 1
        #print("##########################")
        #print("cycle counter:", self.cycle_counter)
        #print("tracker list:", len(self.TrackingObjList))
        #print("##########################")
        self.can_pre = copy.deepcopy(self.can)
        self.data_pre = copy.deepcopy(self.data_)

        self.img, self.data, self.can, self.flg, self.timer, lidar = self.Data.read()

        ref_data = self.data["Ref"]
        del self.data["Ref"]
        #extract tuning mode
        #if self.tuning_mode:
        #    ref_data = self.data["Ref"]
        
        #=======Plot cluster========#
        #base
        plotMovie = self.PlotFigure.get_BaseImage()
        #arrow
        plotMovie = self.PlotFigure.PlotFigure_arrow(plotMovie)
        #velocity
        plotMovie = self.PlotFigure.PlotFigure_CAN(self.can[0].v, plotMovie)
        #lidar
        if "LiDAR" in self.data:
            plotMovie = self.PlotFigure.PlotFigure_lidar(lidar, z_min, z_max, plotMovie)
        #timer
        plotMovie = self.PlotFigure.PlotFigure_t_xavier(self.Data.timer, plotMovie)
        #scaler
        plotMovie = self.PlotFigure.PlotFigure_scale(plotMovie)
        #visualize raw-observed points
        if "SRR520_FL" in self.data:
            self.PlotFigure.PlotFigure_integrated_obs_obj(self.data["SRR520_FL"], plotMovie, (10,255,40), mode="srr520")## Green
        if "SRR520_FR" in self.data:
            self.PlotFigure.PlotFigure_integrated_obs_obj(self.data["SRR520_FR"], plotMovie, (255,230,90), mode="srr520")## Blue
        if "SRR520_RL" in self.data:
            self.PlotFigure.PlotFigure_integrated_obs_obj(self.data["SRR520_RL"], plotMovie, (15,120,235), mode="srr520")## Orange
        if "SRR520_RR" in self.data:
            self.PlotFigure.PlotFigure_integrated_obs_obj(self.data["SRR520_RR"], plotMovie, (200,85,240), mode="srr520")## Purple
        if "ARS510" in self.data:
            self.PlotFigure.PlotFigure_integrated_obs_obj(self.data["ARS510"], plotMovie, (15,230,240), mode="ars510")## Yellow
        if "SVC220" in self.data:
            self.PlotFigure.PlotFigure_integrated_obs_obj(self.data["SVC220"], plotMovie, (200,200,200), mode="svc220")## White
        #if "Ref" in self.data:
        self.PlotFigure.PlotFigure_integrated_obs_obj(ref_data, plotMovie, (255,0,255), mode="reference")## White

        #merged objects
        #PlotFigure.PlotFigure_integrated_obs_obj(integrated_obs, plotMovie, (255,255,255))## Red
        #===========================#


        #ref_data = self.data.pop("Ref")
        self.data_ = copy.deepcopy(self.data)

        #associate with reference data
        #TODO 付加できるものはすべて付加する
        # 未検出対応 ⇒ 追跡物標とマッチングを行い、相関が取れれば検出、取れなければ未検出として保存する
        # 正検出対応 ⇒ 追跡物標とマッチング処理を行い、相関が取れれば検出とする、取れなければ未検出として保存する
        # 誤検出対応 ⇒ 追跡物標のうち、相関が取れなかったものの中でステータスがdetectになっているものを誤検出として保存する

        #Initialize observed-data-list of tracking objects 
        for i in range(len(self.TrackingObjList)):
            obj = self.TrackingObjList[i]
            #print("(0.1)predict: ",self.TrackingObjList[i].prob)
            #print("status:",obj.x, obj.y)
            for s_id in self.TrackingObjList[i].data:
                self.TrackingObjList[i].data[s_id] = []
            self.TrackingObjList[i].Predict(self.can, self.can_pre, self.p_noise)
            obj = self.TrackingObjList[i]
            #print("(0.2)predict: ",self.TrackingObjList[i].prob)
            #print("status:",obj.x, obj.y)



        #Associating objects

        #(1) Associate Tracking objects and Observed objects
        #print("Phase 1: associate with tracking objects")
        addlist = []
        for s_id in self.data:
            
            pairs = []
            if len(self.data[s_id]) > 0:
                p_cov = self.H @ self.p_noise @ self.H.T
                #print("p_noise:", p_cov)
                #print("m_noise:", self.m_noise[s_id])
                pairs = self.Associator.Compute(self.TrackingObjList, self.data[s_id], p_cov, self.m_noise[s_id])
                #    s = input()

            #add matched observed data to track targetlist
            dellist = []
            #print("matched result")
            #print(len(pairs))
            for m_id in range(len(pairs)):
                (i, j) = pairs[m_id]
                self.TrackingObjList[i].data[s_id].append(self.data[s_id][j])
                #dellist.append(self.data[s_id][j])
                dellist.append(j)
                addlist.append(i)

            #delete matched observed data
            print("bofore :", len(self.data[s_id]))
            #for obj in dellist:
            #    self.data[s_id].remove(obj)
            print("after :", len(self.data[s_id]))

            #delete duplicate observed data
            tmp = self.data[s_id]
            self.data[s_id] = []
            for ids in range(len(tmp)):
                if ids not in dellist:
                    self.data[s_id].append(tmp[ids])
            print("after_after :", len(self.data[s_id]))

        #update baysian filter
        minus_index = 0
        deletetracker = []
        for obj_id in range(len(self.TrackingObjList)):
            #print("objs_ids:", obj_id)
            #obj_id = obj_id_- minus_index

            obs = self.TrackingObjList[obj_id].data
            obj = self.TrackingObjList[obj_id]

            #fuse observed points
            if obj_id in addlist:
                obs = self.Fuser.compute(obs, self.m_noise)
                #update
                self.TrackingObjList[obj_id].Update(self.can, self.can_pre, obs, self.m_noise)
                #print("tuisekiseiko!!")
                #s = input()

            L, P = self.BBF.compute(obj, obs_obj=obs, obs_cov=self.m_noise, mode = "update")
            self.TrackingObjList[obj_id].prob = P
            self.TrackingObjList[obj_id].liklihood = L
            obj = self.TrackingObjList[obj_id]
            #print("(1)probab: ",self.TrackingObjList[obj_id].prob)
            #print("status:",obj.x, obj.y)
            #for obs_id in obs:
            #    for ids in obs[obs_id]:
            #        print(ids.x, ids.y)
            #s = input()

            #Existence judgement
            #(1) exist
            if P >= p_exist:
                self.TrackingObjList[obj_id].judge = "exist"
            #(2) hold
            elif p_hold < P < p_exist:
                self.TrackingObjList[obj_id].judge = "hold"
            #(3) misdetect or lost
            else: 
                self.TrackingObjList[obj_id].judge = "delete"
                #deletetracker.append(self.TrackingObjList[obj_id])
                deletetracker.append(obj_id)
                #obj_id -= 1
                #minus_index += 1
        
        #for obj in deletetracker:
        #    self.TrackingObjList.remove(obj)

        #delete duplicate observed data
        tmp = self.TrackingObjList
        self.TrackingObjList = []
        for ids in range(len(tmp)):
            if ids not in deletetracker:
                self.TrackingObjList.append(tmp[ids])
        print("after_after :", len(self.data[s_id]))


        #(2) Associate observed data that did not matched with tracking objects
        #print("Phase 2: associate with observed data")
        Det = [ids for ids in self.data]
        Det_ = [ids for ids in self.data]
        print(Det)
        print(Det_)
        #Det  = {"ARS510":[], "SRR520_FL":[], "SRR520_FR":[], "SRR520_RL":[], "SRR520_RR":[]}#, "SVC220":[]}
        #Det_ = {"ARS510":[], "SRR520_FL":[], "SRR520_FR":[], "SRR520_RL":[], "SRR520_RR":[]}#, "SVC220":[]}
        #frame = ["SVC220", "ARS510", "SRR520_FL", "SRR520_FR", "SRR520_RL", "SRR520_RR"]
        init_point = len(self.TrackingObjList)
        for A_id in Det:

            #delete sensor ids from Det_
            #del Det_[A_id]
            Det_.remove(A_id)

            for B_id in Det_:
                
                #(2.1) associate with matched obj-list
                dellist = []
                #for s_id in frame:
                objA = self.TrackingObjList
                objB = self.data[B_id]
                if len(objB) > 0 and len(objA) > 0:
                    p_cov = self.H @ self.p_noise @ self.H.T
                    pairs = self.Associator.Compute(objA, objB, p_cov, self.m_noise[B_id])

                    for m_id in range(len(pairs)):
                        (i, j) = pairs[m_id]
                        self.TrackingObjList[i].data[B_id].append(self.data[B_id][j])
                        dellist.append(self.data[B_id][j])

                #delete matched observed data
                for obj in dellist:
                    self.data[B_id].remove(obj)

                #(2.2) associate with observed data
                #associate
                dellistA = []
                dellistB = []
                objA = self.data[A_id]
                objB = self.data[B_id]
                if len(objA) > 0 and len(objB) > 0:
                    pairs = self.Associator.Compute(objA, objB, self.m_noise[A_id], self.m_noise[B_id])


                    for m_id in range(len(pairs)):
                        (i, j) = pairs[m_id]
                        #ressiter objects(reinit tracker later..)
                        obs = {B_id:[objB[j]], A_id:[objA[i]], "fused":[]}
                        #new = Tracker(objB[j], self.can, mode=self.track_mode)
                        new = Tracker(obs, self.can, mode=self.track_mode)
                        self.TrackingObjList.append(new)
                        #self.TrackingObjList += Tracker(objB[j], self.can, mode=self.track_mode)
                        self.TrackingObjList[-1].data[A_id].append(objA[i])
                        self.TrackingObjList[-1].data[B_id].append(objB[j])
                        #memory delete obj data
                        #dellistA.append(self.data[A_id][i])
                        #dellistB.append(self.data[B_id][j])
                        dellistA.append(i)
                        dellistB.append(j)

                #delete matched observed data
                #for obj in dellistA:
                #    self.data[A_id].remove(obj)
                #for obj in dellistB:
                #    self.data[B_id].remove(obj)

                #delete duplicate observed data
                tmpA = self.data[A_id]
                tmpB = self.data[B_id]
                self.data[A_id] = []
                self.data[B_id] = []
                for ids in range(len(tmpA)):
                    if ids not in dellistA:
                        self.data[A_id].append(tmpA[ids])
                for ids in range(len(tmpB)):
                    if ids not in dellistB:
                        self.data[B_id].append(tmpB[ids])

        #update baysian filter
        if len(self.TrackingObjList) > 0:
            for obj_id in range(init_point, len(self.TrackingObjList)):

                obs = self.TrackingObjList[obj_id].data
                #obj = self.TrackingObjList[obj_id]

                #fuse observed points
                obs = self.Fuser.compute(obs, self.m_noise)
                #print("(2.1)probab: ",self.TrackingObjList[obj_id].prob )
                #print("status:",obj.x, obj.y)

                #update -> reinit
                self.TrackingObjList[obj_id].InitTracker(obs, self.can)
                L, P = self.BBF.compute([], obs_obj=obs, obs_cov=self.m_noise, mode = "measurement")
                self.TrackingObjList[obj_id].prob = P
                self.TrackingObjList[obj_id].liklihood = L

                #Existence judgement
                #(1) exist
                if P >= p_exist:
                    self.TrackingObjList[obj_id].judge = "exist"
                #(2) hold
                elif P < p_exist:
                    self.TrackingObjList[obj_id].judge = "hold"

                #obj = self.TrackingObjList[obj_id]
                #print("(2.2)probab: ",self.TrackingObjList[obj_id].prob )
                #print("status:",obj.x, obj.y)
                #for obs_id in obs:
                #    for ids in obs[obs_id]:
                #        print(ids.x, ids.y)
                #s = input()

            

        #(3) Add observed data that didn't mathced with any obj as new objects
        #print("Phase.3 add as an TBD obj")
        for s_id in self.data:
            for obs_id in range(len(self.data[s_id])):
                
                #init tracker
                obj = self.data[s_id][obs_id]
                obs = {s_id:[self.data[s_id][obs_id]], "fused":[]}
                new = Tracker(obs, self.can, mode=self.track_mode)
                #self.TrackingObjList.append(Tracker(obs, self.can, mode=self.track_mode))
                self.TrackingObjList.append(new)
                
                #init probability
                self.TrackingObjList[-1].prob = p_hold
                self.TrackingObjList[-1].liklihood = 0.0
                self.TrackingObjList[-1].judge = "hold"

                #obj = self.TrackingObjList[-1]
                #print("(3)probab: ",self.TrackingObjList[-1].prob )
                #print("status:",obj.x, obj.y)
                #s = input()

        #(4)Delete duplicate objects
        self.TrackingObjList = self.Delete_Duplication(self.TrackingObjList)

        ###########################"
        # tracking obejcts check!!!"
        ###########################"
        output = []
        for obj in self.TrackingObjList:
            if obj.judge == "exist":
                output.append(obj)
        self.PlotFigure.PlotFigure_integrated_obs_obj(output, plotMovie,color=(0,0,255))## Red
        self.PlotFigure.showImage(plotMovie)


        

        #if self.tuning_mode:
        #TODO tuning processnoise to improve accuracy
        
        #step.0 associate tracking objects with regerence objects
        ############################################################
        ## Return residual
        ############################################################
        if(len(ref_data) > 0 and self.tuning_mode == 1):
            #prepare
            cost = np.zeros((4,1))
            residual = np.zeros((4,1))
            match_tracker = []
            match_ref = []

            obj = self.TrackingObjList
            ref = ref_data
            pairs = self.Associator.Compute(obj, ref, self.m_noise["SVC220"], self.m_noise["SVC220"], mode="inverse")
            
            #step.0.1 : calculate cost and residual with matched objects
            for (i,j) in pairs:
                #cost
                cost += np.array([
                                  [(ref[j].x - obj[i].x)**2.0],
                                  [(ref[j].y - obj[i].y)**2.0],
                                  [(ref[j].w - obj[i].w)**2.0],
                                  [(ref[j].l - obj[i].l)**2.0]
                                ])
                #residual
                residual += np.array([
                                      [ref[j].x - obj[i].x],
                                      [ref[j].y - obj[i].y],
                                      [ref[j].w - obj[i].w],
                                      [ref[j].l - obj[i].l]
                                     ])

                match_tracker.append(i)
                match_ref.append(j)
                #print("res shape:", residual.shape)

            #step.0.2 : calculate cost and add to cost_
            for j in range(len(ref)):
                if j not in match_ref:
                    #最近傍点と比較する
                    #TODO : how???
                    index = 0
                    d_min = 100
                    for i in range(len(obj)):
                        d = (obj[i].x - ref[j].x)**2.0 + (obj[i].y - ref[j].y)**2.0
                        if d < d_min:
                            index = i
                            d_min = d

                    #cost
                    if ref[j].ref_cls != 0:
                        cost += np.array([
                                          [(ref[j].x - obj[index].x)**2.0],
                                          [(ref[j].y - obj[index].y)**2.0],
                                          [(ref[j].w - obj[index].w)**2.0],
                                          [(ref[j].l - obj[index].l)**2.0]
                                        ])
                        #residual
                        residual += np.array([
                                              [ref[j].x - obj[index].x],
                                              [ref[j].y - obj[index].y],
                                              [ref[j].w - obj[index].w],
                                              [ref[j].l - obj[index].l]
                                             ])
                        #print("res_x:", ref[j].x - obj[index].x)
                        #print("res_y:", ref[j].y - obj[index].y)

            #return cost, residual
        else:
            cost = np.zeros((4,1))
            residual = np.zeros((4,1))
            #return cost, residual
                


        
        ############################################################
        ## Return residual
        ############################################################
        if self.args.evaluator:
            if len(ref_data) > 0:
                #open file
                with open("../raw_data/" + self.args.data_name + "result_tracker=" + self.args.tracker + "_asso="+self.args.associator+"_xtss="+self.args.xtss+"_mode="+str(self.args.tuning)+".csv", mode="a", newline="")as f:
                    writer = csv.writer(f)
                    #(4) Evaluate accuracy and detection rate
                    obj = self.TrackingObjList
                    ref = ref_data
                    dellist = []
                    poplist = []
                    for ids in range(len(ref)):
                        if ref[ids].ref_x in dellist:
                            poplist.append(ids)
                        else:
                            dellist.append(ref[ids].ref_x)
                    for ids in sorted(poplist, reverse=True):
                        ref.pop(ids)


                    pairs = self.Associator.Compute(obj, ref, self.m_noise["SVC220"], self.m_noise["SVC220"], mode="inverse")
                    #print(pairs)

                    #(4.1) detect
                    m_s = []
                    m_r = []
                    for i, j in pairs:
                        m_s.append(i)
                        m_r.append(j)

                    for i, j in pairs:
                        #save
                        if obj[i].judge == "exist":
                            status = "detect"
                            timer = self.timer
                            obs_id, obs_x, obs_y, obs_w, obs_l, obs_theta, obs_class = 0, obj[i].x, obj[i].y, obj[i].w, obj[i].l, obj[i].o, "unknown"
                            status, sense_id, tar_x, tar_y, tar_w, tar_l, tar_theta, tar_class = status, "track", ref[j].ref_x, ref[j].ref_y, ref[j].ref_w, ref[j].ref_l, ref[j].o, ref[j].cls
                            
                            datalist = [timer, tar_x, tar_y, tar_w, tar_l, tar_theta, tar_class, status, sense_id, obs_x, obs_y, obs_w, obs_l, obs_theta, obs_class, obj[i].prob]
                            writer.writerow(datalist)

                        elif obj[i].judge == "hold" or obj[i].judge == "delete":
                            status = "undetect"
                            timer = self.timer
                            obs_id, obs_x, obs_y, obs_w, obs_l, obs_theta, obs_class = 0, obj[i].x, obj[i].y, obj[i].w, obj[i].l, obj[i].o, "unknown"
                            status, sense_id, tar_x, tar_y, tar_w, tar_l, tar_theta, tar_class = status, "track", ref[j].ref_x, ref[j].ref_y, ref[j].ref_w, ref[j].ref_l, ref[j].o, ref[j].cls
                            
                            datalist = [timer, tar_x, tar_y, tar_w, tar_l, tar_theta, tar_class, status, sense_id, obs_x, obs_y, obs_w, obs_l, obs_theta, obs_class, obj[i].prob]
                            writer.writerow(datalist)


                    #(4.2) misdetect
                    for ids in range(len(self.TrackingObjList)):
                        if ids not in m_s:
                            status = "missdetect"
                            #save with status
                            if obj[ids].judge == "exist":
                                status = "missdetect"
                                timer = self.timer
                                obs_id, obs_x, obs_y, obs_w, obs_l, obs_theta, obs_class = 0, obj[ids].x, obj[ids].y, obj[ids].w, obj[ids].l, obj[ids].o, "unknown"
                                status, sense_id, tar_x, tar_y, tar_w, tar_l, tar_theta, tar_class = status, "track", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                            
                                datalist = [timer, tar_x, tar_y, tar_w, tar_l, tar_theta, tar_class, status, sense_id, obs_x, obs_y, obs_w, obs_l, obs_theta, obs_class, obj[ids].prob]
                                writer.writerow(datalist)

                    #(4.3)undetect
                    for ids in range(len(ref)):
                        if ids not in m_r:
                            status = "undetect"
                            #savestatus = "missdetect"
                            timer = self.timer
                            obs_id, obs_x, obs_y, obs_w, obs_l, obs_theta, obs_class = 0, 0, 0, 0, 0, 0, "unknown"
                            status, sense_id, tar_x, tar_y, tar_w, tar_l, tar_theta, tar_class = "undetect", "track", ref[ids].ref_x, ref[ids].ref_y, ref[ids].ref_w, ref[ids].ref_l, ref[ids].o, ref[ids].cls
                            
                            datalist = [timer, tar_x, tar_y, tar_w, tar_l, tar_theta, tar_class, status, sense_id, obs_x, obs_y, obs_w, obs_l, obs_theta, obs_class, 0.0]
                            writer.writerow(datalist)

        return cost, residual


    ##=========================================
    ## Delete duplicated tracking ofject
    ##=========================================
    def Delete_Duplication(self, tracking_obj_list):
        non_duplication_trk_list = []
        x_lim = 5
        y_lim = 2

        while len(tracking_obj_list) != 0:
            #print('len ', len(tracking_obj_list.obj))
            duplication_list = []
            del_list = []
            pop0 = tracking_obj_list.pop(0)
            duplication_list.append(pop0)
            x0 = pop0.x
            y0 = pop0.y
            
            for i in range(len(tracking_obj_list)):
                
                x = tracking_obj_list[i].x
                y = tracking_obj_list[i].y
                
                if x0-x_lim < x and x < x0+x_lim and y0-y_lim < y and y < y0+y_lim:
                    
                    temp = tracking_obj_list[i]
                    duplication_list.append(temp)
                    del_list.append(i)
            
            #one = self.local_integrate_by_average(duplication_list)
            
            one = duplication_list[0]
            #one = self.local_integrate_by_single(duplication_list)
            non_duplication_trk_list.append(one)

            #print(del_list, len(duplication_list))

            for d in sorted(del_list, reverse=True):
                del tracking_obj_list[d]

        return non_duplication_trk_list

    ##=========================================
    ## Integrate duplicated tracking ofject in local area
    ##=========================================
    def local_integrate_by_average(self, duplication_list):
        x = 0
        y = 0
        vx = 0
        vy = 0
        w = 0
        l = 0
        for i in range(len(duplication_list)):
            #x_s += duplication_list[i].x_s
            #P_s += duplication_list[i].P_s
            x  += duplication_list[i].x
            y  += duplication_list[i].y
            vx += duplication_list[i].vx
            vy += duplication_list[i].vy
            w  += duplication_list[i].w
            l  += duplication_list[i].l
        sums = len(duplication_list)
        #duplication_list[0].tracker.x  += x  / sums
        #duplication_list[0].tracker.y  += y  / sums
        #duplication_list[0].tracker.vx += vx / sums
        #duplication_list[0].tracker.vy += vy / sums
        #duplication_list[0].tracker.w  += w  / sums
        #duplication_list[0].tracker.l  += l  / sums
        #print("sums:", duplication_list[i].x)

        return duplication_list[0]


