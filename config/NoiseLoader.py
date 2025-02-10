from cv2 import dnn_DetectionModel
import numpy as np
from numpy import cos, sin, arctan2, pi, tan, sqrt
from numpy.core.overrides import verify_matching_signatures

class NoiseLoader:
	def __init__(self):
		self.data = None
	
	def load(self, filename):
		data = np.load(filename)
		try:
			return data.files, data
		except:
		    return None, data

	def save(self, filename, data):
		np.save(filename, data)

	def savez(self, filename, data=[]):
		np.save(filename, data)

if __name__ == "__main__":
    #x,y,vx,vy,w,l
	
    cov = np.identity(6) * 2.0
    p_cov = np.identity(8) * 2.0
    #camera noise
    svc_cov = np.array([[2.0**2.0, 0.0, 0.0, 0.0, 0.0, 0.0], #x
                        [0.0, 0.5**2.0, 0.0, 0.0, 0.0, 0.0], #y
                        [0.0, 0.0, 2.0**2.0, 0.0, 0.0, 0.0], #vx
                        [0.0, 0.0, 0.0, 2.0**2.0, 0.0, 0.0], #vy
                        [0.0, 0.0, 0.0, 0.0,  0.9**2.0, 0.0], #w
                        [0.0, 0.0, 0.0, 0.0, 0.0, 1.5**2.0]])#l
    #ARS cov
    ars_cov = np.array([[1.0**2.0, 0.0, 0.0, 0.0, 0.0, 0.0], #x
                        [0.0, 2.0**2.0, 0.0, 0.0, 0.0, 0.0], #y
                        [0.0, 0.0, 2.0**2.0, 0.0, 0.0, 0.0], #vx
                        [0.0, 0.0, 0.0, 2.0**2.0, 0.0, 0.0], #vy
                        [0.0, 0.0, 0.0, 0.0,  0.5**2.0, 0.0], #w
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.5**2.0]])#l

    #srr cov
    srr_cov = np.array([[1.5**2.0, 0.0, 0.0, 0.0, 0.0, 0.0], #x
                        [0.0, 2.5**2.0, 0.0, 0.0, 0.0, 0.0], #y
                        [0.0, 0.0, 2.5**2.0, 0.0, 0.0, 0.0], #vx
                        [0.0, 0.0, 0.0, 2.5**2.0, 0.0, 0.0], #vy
                        [0.0, 0.0, 0.0, 0.0,  0.5**2.0, 0.0], #w
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.5**2.0]])#l

    #save each sensor noise sensor
    Loader = NoiseLoader()
    Loader.save("../config/svc220_cov.npy", svc_cov)
    Loader.save("../config/ars510_cov.npy", ars_cov)
    Loader.save("../config/srr520_cov.npy", srr_cov) #TODO verify accuracy and reset. And init covariance wtih each sensors
    print("SVCcov: ", svc_cov)
    print("arscov: ", ars_cov)
    print("srrcov: ", srr_cov)

    #Loader.save("../config/m_noise_cov.npy", cov * 2)
    #Loader.save("../config/p_noise_cov.npy", p_cov)
    #np.savez("../config/m_noise_cov.npz", ARS510=mcov, SVC220=mcov*2.0)

    noise = Loader.load("../config/p_noise_cov.npy")
    mnoise = Loader.load("../config/m_noise_cov.npy")
    sensors, mnosie = Loader.load("../config/m_noise_cov.npz")
    print(noise)
    print(mnoise)
    print(sensors)
    for id in sensors:
        print(mnosie[id])
	
