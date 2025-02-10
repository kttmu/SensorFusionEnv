import struct
import numpy as np
import math

Lidar_MAC_src = [b'\x60',b'\x76',b'\x88',b'\x10',b'\x4b',b'\x6c']
Lidar_MAC_dst = [b'\xff',b'\xff',b'\xff',b'\xff',b'\xff',b'\xff']

DataPackets  = np.empty((66,12))
DataPackets2 = np.empty((17,24))
# PosX = np.zeros(28416)
# PosY = np.zeros(28416)
# PosZ = np.zeros(28416)
# Reflct = np.zeros(28416)

PosX = np.zeros(29184)
PosY = np.zeros(29184)
PosZ = np.zeros(29184)
Reflct = np.zeros(29184)

Omega_ref = [-25,-1,-1.667,-15.639,-11.31,0,-0.667,-8.843,-7.254,0.333,-0.333,-6.148,-5.333,1.333,0.667,-4,-4.667,1.667,1,-3.667,-3.333,3.333,2.333,-2.667,-3,7,4.667,-2.333,-2,15,10.333,-1.333]

Azimuth_Offset = [1.4,-4.2,1.4,-1.4,1.4,-1.4,4.2,-1.4,1.4,-4.2,1.4,-1.4,4.2,-1.4,4.2,-1.4,1.4,-4.2,1.4,-4.2,4.2,-1.4,1.4,-1.4,1.4,-1.4,1.4,-4.2,4.2,-1.4,1.4,-1.4]

# ================================================================= #
class VLP32:

    # ============================================================= #
    def __init__(self):
        pass

    # ============================================================= #
    def MAC_address_lidar(self):
        MAC_address = Lidar_MAC_dst + Lidar_MAC_src
        return MAC_address

    # ============================================================= #
    def ReadLidar(self,fileUDP):
        for i in range(12):
            pkts = fileUDP[i*100:(i+1)*100]
            

            tmp = struct.unpack("<HHHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHB", pkts)
            
            for j in range(66):
                DataPackets[j,i] = tmp[j]
        
        azmth  = self.ReadAzimuth(DataPackets)
        dist   = self.ReadDistance(DataPackets)
        reflct = self.ReadReflectivity(DataPackets)

        posX,posY,posZ = self.Calc_posXYZ(azmth,dist)
        
        
        # PosX[0:28032] = PosX[384:28416]
        # PosY[0:28032] = PosY[384:28416]
        # PosZ[0:28032] = PosZ[384:28416]
        # PosX[28032:28416] = posX
        # PosY[28032:28416] = posY
        # PosZ[28032:28416] = posZ

        # Reflct[0:28032] = Reflct[384:28416]
        # Reflct[28032:28416] = reflct

        PosX[0:28800] = PosX[384:29184]
        PosY[0:28800] = PosY[384:29184]
        PosZ[0:28800] = PosZ[384:29184]
        PosX[28800:29184] = posX
        PosY[28800:29184] = posY
        PosZ[28800:29184] = posZ

        Reflct[0:28800] = Reflct[384:29184]
        Reflct[28800:29184] = reflct

        pkts = fileUDP[1200:1206]
        pkts_xavier = fileUDP[1210:1218]
        tmp = struct.unpack("<IBB", pkts)
        tmp_x = struct.unpack("<Q", pkts_xavier)
        timeStamp = tmp[0]
        factoryBytes1 = tmp[1]
        factoryBytes2 = tmp[2]

        timeStamp_xavier = tmp_x[0]
     

        return PosX,PosY,PosZ,Reflct,timeStamp,timeStamp_xavier


    # ============================================================= #
    def ReadAzimuth(self,DataPackets):
        azmth = np.empty(12)
        azmth[0:12] = DataPackets[1,:]

        for i in range(12):
            if azmth[i] >= 36000: 
                azmth[i] -= 36000

        if azmth[9] < azmth[10]:
            azmth[11] = 2*azmth[10] - azmth[9]
        else:
            azmth[11] = 2*azmth[10] - azmth[9] + 36000
        
        if azmth[11] >= 36000:
            azmth[11] -= 36000

        azmth /= 100
        return azmth

    # ============================================================= #
    def ReadDistance(self,DataPackets):
        dist = np.empty((32,12))
        dist[:,0:12] = DataPackets[ 2:66:2,:]

        dist *= 0.004
        return dist

    # ============================================================= #
    def ReadReflectivity(self,DataPackets):
        reflct = np.empty((32,12))
        reflct[:,0:12] = DataPackets[ 3:67:2,:]
        reflct = reflct.reshape(-1)

        return reflct

    # ============================================================= #
    def Calc_posXYZ(self,azmth,dist):

        alpha = azmth * math.pi/180
        delta = np.array(Azimuth_Offset)*math.pi/180
        delta = np.reshape(delta,(-1,1))
        
        omega = np.array(Omega_ref) * math.pi/180
        omega = np.reshape(omega,(-1,1))

        ###--Hosei (theta = alpha + delta)
        theta = np.tile(alpha,(32,1)) + delta

        posX = dist * np.cos(omega) * np.sin(theta)
        posY = dist * np.cos(omega) * np.cos(theta)
        posZ = dist * np.sin(omega)

        posX = posX.reshape(-1)
        posY = posY.reshape(-1)
        posZ = posZ.reshape(-1)

        return posX,posY,posZ