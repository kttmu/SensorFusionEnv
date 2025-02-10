import struct
import numpy as np
import math

Lidar_MAC_src = [b'\x60',b'\x76',b'\x88',b'\x10',b'\x4b',b'\x6c']
Lidar_MAC_dst = [b'\xff',b'\xff',b'\xff',b'\xff',b'\xff',b'\xff']


DataPackets  = np.empty((66,12))
DataPackets2 = np.empty((17,24))
PosX = np.zeros(14592)
PosY = np.zeros(14592)
PosZ = np.zeros(14592)
Reflct = np.zeros(14592)

Omega_ref = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]

# ================================================================= #
class VLP16:

    # ============================================================= #
    def __init__(self):
        pass

    # ============================================================= #
    def MAC_address_lidar(self):
        MAC_address = Lidar_MAC_dst + Lidar_MAC_src
        return MAC_address

    # ============================================================= #
    def ReadLidar(self,fileUDP):
        pointcloud_sph = np.empty((1,4))
        for i in range(12):
            pkts = fileUDP[i*100:(i+1)*100]
            tmp = struct.unpack("<HHHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHB", pkts)

            for j in range(66):
                DataPackets[j,i] = tmp[j]
        
        azmth  = self.ReadAzimuth(DataPackets)
        dist   = self.ReadDistance(DataPackets)
        reflct = self.ReadReflectivity(DataPackets)

        posX,posY,posZ = self.Calc_posXYZ(azmth,dist)
        
        PosX[0:14208] = PosX[384:14592]
        PosY[0:14208] = PosY[384:14592]
        PosZ[0:14208] = PosZ[384:14592]
        PosX[14208:14592] = posX
        PosY[14208:14592] = posY
        PosZ[14208:14592] = posZ

        Reflct[0:14208] = Reflct[384:14592]
        Reflct[14208:14592] = reflct

        pkts = fileUDP[1200:1206]
        tmp = struct.unpack("<IBB", pkts)
        timeStamp = tmp[0]
        factoryBytes1 = tmp[1]
        factoryBytes2 = tmp[2]

        return PosX,PosY,PosZ,Reflct,timeStamp
   
    # ============================================================= #
    def ReadAzimuth(self,DataPackets):
        azmth = np.empty(24)
        azmth[0:24:2] = DataPackets[1,:]
        
        for i in range(1,22,2):
            if azmth[i-1] < azmth[i+1]:
                azmth[i] =  (azmth[i-1] + azmth[i+1])/2
            else:
                azmth[i] =  (azmth[i-1] + azmth[i+1]+36000)/2
                if azmth[i] >= 36000:
                    azmth[i] -= 36000

        if azmth[21] < azmth[22]:
            azmth[23] = azmth[22] + (azmth[22] - azmth[21])
        else:
            azmth[23] = 2*azmth[22] - azmth[21] + 36000
        
        if azmth[23] >= 36000:
            azmth[23] -= 36000

        azmth /= 100
        return azmth
            
    # ============================================================= #
    def ReadDistance(self,DataPackets):
        dist = np.empty((16,24))
        dist[:,0:24:2] = DataPackets[ 2:34:2,:]
        dist[:,1:24:2] = DataPackets[34:66:2,:]

        dist *= 0.002
        return dist

    # ============================================================= #
    def ReadReflectivity(self,DataPackets):
        reflct = np.empty((16,24))
        reflct[:,0:24:2] = DataPackets[ 3:35:2,:]
        reflct[:,1:24:2] = DataPackets[35:67:2,:]
        reflct = reflct.reshape(-1)

        return reflct

    # ============================================================= #
    def Calc_posXYZ(self,azmth,dist):

        alpha = azmth * math.pi/180
        omega = np.array(Omega_ref) * math.pi/180
        omega = np.reshape(omega,(-1,1))

        # print(azmth)
        # print(alpha.shape)
        # print(dist.shape)
        # print(omega.shape)

        posX = dist * np.cos(omega) * np.sin(alpha)
        posY = dist * np.cos(omega) * np.cos(alpha)
        posZ = dist * np.sin(omega)

        posX = posX.reshape(-1)
        posY = posY.reshape(-1)
        posZ = posZ.reshape(-1)

        return posX,posY,posZ
