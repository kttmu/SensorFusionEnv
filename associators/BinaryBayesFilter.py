import numpy  as np
import numpy.linalg as LA
from numpy.linalg import inv
import json
#from ..config import p_noise_cov
p_max = 0.5
p_thresh = 0.2
obs_max = 0.2
obs_thresh = 0.8

#TODO in this class
# get the meas data
# get the covariance data
# set target sensors
#  
class BBF:
	def __init__(self):
		#set cov
		file = open("./config/p_noise_cov.json")
		self.cov_info = json.load(file)
		print(self.cov_info)
		#s = input()
		#self.cov = {}
		#self.init_cov()
		self.cov_mode = "SVC220"
		self.xhi_val = 13.3 #if obsed dim == 6, set 18.6 REF:http://www3.u-toyama.ac.jp/kkarato/2019/statistics/table/chisq.pdf 

	def init_cov(self):
		for s_id in self.cov_info:
			elems = np.zeros(6)
			elems[0] = self.cov_info[s_id]["x"]
			elems[1] = self.cov_info[s_id]["y"]
			elems[2] = self.cov_info[s_id]["vx"]
			elems[3] = self.cov_info[s_id]["vy"]
			elems[4] = self.cov_info[s_id]["w"]
			elems[5] = self.cov_info[s_id]["l"]

			self.cov[s_id] = np.zeros((4,4))
			for i in range(4):
				self.cov[s_id][i,i] = elems[i]

    #======================================================#
	# function : compute prob_exists
	# input : target tracked objects and target obs_obj, and compute mode "update" and "meased"
	# output: odds and probability(or should i call cost)
    #======================================================#
	def compute(self, track_obj, obs_obj={}, obs_cov=None, mode = "update"):
		L = 0
		P = 0

		#============Measurement mode=================#
		#det = {"SVC220.ALL":"SVC220", "ARS510.FC":"ARS510"}
		#print("prob and L", P, L, mode)

		if mode == "measurement":
			mini_cov = "SVC220"
			for s_id in obs_obj:
				for obs_id in range(len(obs_obj[s_id])):
					#if obs_cov[det[s_id]][0,0] <= obs_cov[det[mini_cov]][0,0]:
					if obs_cov[s_id][0,0] <= obs_cov[mini_cov][0,0]:
						mini_cov = s_id

						#BBF
						#zc = obs_obj[mini_cov][0].state
						#zc[0,0] = obs_obj[mini_cov][0].x
						zc = np.zeros((6,1))
						zc[0,0] = obs_obj[mini_cov][0].x
						zc[1,0] = obs_obj[mini_cov][0].y
						zc[2,0] = obs_obj[mini_cov][0].vx
						zc[3,0] = obs_obj[mini_cov][0].vy
						zc[4,0] = obs_obj[mini_cov][0].w
						zc[5,0] = obs_obj[mini_cov][0].l
			
			L = 0 #or - infinity????
			for s_id in obs_obj:
				for obs_id in range(len(obs_obj[s_id])):
					if not (s_id == mini_cov and obs_id == 0):
						#z = obs_obj[s_id][obs_id].state
						z = np.zeros((6,1))
						z[0,0] = obs_obj[s_id][obs_id].x
						z[1,0] = obs_obj[s_id][obs_id].y
						z[2,0] = obs_obj[s_id][obs_id].vx
						z[3,0] = obs_obj[s_id][obs_id].vy
						z[4,0] = obs_obj[s_id][obs_id].w
						z[5,0] = obs_obj[s_id][obs_id].l
						d = (z - zc).T @ inv(obs_cov[s_id]) @ (z - zc)
						if d <= self.xhi_val:
							p = 0.9
							#p = obs_thresh + (obs_max - obs_thresh) * np.exp(-d / 2)
							#p = 0
							L += np.log(p / (1 - p))
			
			#exisistant probability
			P = float(np.exp(L) / (1 + np.exp(L)))
			#print("prob and L", mini_cov, P, L)
        
		#==============Update mode===================#
		elif mode == "update":

			#mini_cov = "SVC220"
			#for s_id in obs_obj:
			#	for obs_id in range(len(obs_obj[s_id])):
			#		if obs_cov[s_id][0,0] <= obs_cov[mini_cov][0,0]:
			#			mini_cov = s_id

			#			#observation filtering
			#			#zc = obs_obj[mini_cov][0].state
			#			zc = np.zeros((6,1))
			#			zc[0,0] = obs_obj[mini_cov][obs_id].x
			#			zc[1,0] = obs_obj[mini_cov][obs_id].y
			#			zc[2,0] = obs_obj[mini_cov][obs_id].vx
			#			zc[3,0] = obs_obj[mini_cov][obs_id].vy
			#			zc[4,0] = obs_obj[mini_cov][obs_id].w
			#			zc[5,0] = obs_obj[mini_cov][obs_id].l
			
			#L_obs = 0 #or - infinity????
			#for s_id in obs_obj:
			#	for obs_id in range(len(obs_obj[s_id])):
			#		if not (s_id == mini_cov and obs_id == 0):
			#			#z = obs_obj[s_id][obs_id].state
			#			z = np.zeros((6,1))
			#			z[0,0] = obs_obj[mini_cov][obs_id].x
			#			z[1,0] = obs_obj[mini_cov][obs_id].y
			#			z[2,0] = obs_obj[mini_cov][obs_id].vx
			#			z[3,0] = obs_obj[mini_cov][obs_id].vy
			#			z[4,0] = obs_obj[mini_cov][obs_id].w
			#			z[5,0] = obs_obj[mini_cov][obs_id].l
			#			d = (z - zc).T @ inv(obs_cov[s_id]) @ (z - zc)
			#			if d < self.xhi_val:
			#				p = obs_thresh + (obs_max - obs_thresh) * np.exp(-d / 2)
			#			    #p = np.exp(-d / 2)
			#			    #p = 0.65
			#				#p = 0
			#				L_obs += np.log(p / (1 - p))
			
			##exisistant probability
			#P_obs = float(np.exp(L_obs) / (1 + np.exp(L_obs)))


			#merge with past tracking information
			#TODO consider IMM method
			cov = track_obj.tracker.p_cv
			p = []
			L_t = 0
			#print("obs obj in:", obs_obj)
			if len(obs_obj["fused"]) > 0:
				z = np.zeros((6,1))
				z[0,0] = obs_obj["fused"][0].x
				z[1,0] = obs_obj["fused"][0].y
				z[2,0] = obs_obj["fused"][0].vx
				z[3,0] = obs_obj["fused"][0].vy
				z[4,0] = obs_obj["fused"][0].w
				z[5,0] = obs_obj["fused"][0].l

				x = track_obj.tracker.s_ca
				H = track_obj.tracker.MeasMatrix(x, z)
				S = obs_cov["SVC220"] + H @ cov @ H.T
				d = (z - H @ x).T @ inv(S) @ (z - H @ x)
				#p = p_thresh + (p_max - p_thresh) * np.exp(-d / 2)
				#p.append(obs_thresh + (obs_max - obs_thresh) * np.exp(-d / 2))
				p.append(0.9)
				#p = 0
				#p = 0.8
			else:
				for s_id in obs_obj:
					for i in range(len(obs_obj[s_id])):
						z = np.zeros((6,1))
						z[0,0] = obs_obj[s_id][i].x
						z[1,0] = obs_obj[s_id][i].y
						z[2,0] = obs_obj[s_id][i].vx
						z[3,0] = obs_obj[s_id][i].vy
						z[4,0] = obs_obj[s_id][i].w
						z[5,0] = obs_obj[s_id][i].l
						x = track_obj.tracker.s_ca
						H = track_obj.tracker.MeasMatrix(x, z)

						S = obs_cov["SVC220"] + H @ cov @ H.T
						d = (z - H @ x).T @ inv(S) @ (z - H @ x)
						#p = p_thresh + (p_max - p_thresh) * np.exp(-d / 2)
						#p.append(obs_thresh + (obs_max - obs_thresh) * np.exp(-d / 2))
						p.append(0.9)
						#p = 0
						#p = 0.7
						#L_t = np.log(p / (1 - p))
			if len(p) ==0:
				p.append(0.05)
			for prob in p:
				L_t += np.log(prob / (1 - prob))
			L = track_obj.liklihood + L_t# - np.log(0.55 / (1 - 0.55))# + L_obs
			P = float(np.exp(L) / (1 + np.exp(L)))
			track_obj.prob = P
			#print(track_obj.tracker.L)
			track_obj.liklihood = L
			#print("prob;",P)
		
		elif mode == "failure":
			#ここでのインプットは観測と追跡物標の辞書でなければならない
			match_cam = 0
			math_fail_cam = 0
			match_suc = {"SVC220":0, "ARS510":0}
			match_tar = {"SVC220":0, "ARS510":0}

			for tar_id in track_obj:
				obj = track_obj[tar_id]
				for s_id in obs_obj:
					match_tar[s_id] += 1
					for ids in obs_obj[s_id]:
						match_suc[s_id] += 1
		
			L = 0 #or - infinity????

			for s_id in match_suc:
				p = match_suc[s_id] / match_tar[s_id]
				L += np.log(p / (1 - p))
			
			#exisistant probability
			P = float(np.exp(L) / (1 + np.exp(L)))
			#print("prob and L", P, L)

			pass


		return L, P
				


if __name__ == "__main__":
	logit = {
		"SVC220":{"x":10.0, "y":10.0, "z":10.0, "vx":10.0, "vy":10.0, "w":10.0, "l":10.0},
	    "ARS510":{"x":10.0, "y":10.0, "z":10.0, "vx":10.0, "vy":10.0, "w":10.0, "l":10.0},
	    "SVC220.ALL":{"x":3.0, "y":2.0, "vx":3.0, "vy":2.0, "w":1.0, "l":1.0},
	    "ARS510.FC":{"x":0.50, "y":1.0, "vx":3.0, "vy":2.0, "w":1.0, "l":1.0},
	    "SAMPLE":{"x":100.0, "y":100.0, "z":100.0, "vx":100.0, "vy":100.0, "w":100.0, "l":100.0},
	    "UNI":{"x":1.0, "y":1.0, "z":1.0, "vx":1.0, "vy":1.0, "w":1.0, "l":1.0}
	}
	json_file = open("../config/m_noise_cov.json", "w")
	json.dump(logit, json_file)
	json_file = open("../config/m_noise_cov.json", "r")
	data = json.load(json_file)
	print(data)