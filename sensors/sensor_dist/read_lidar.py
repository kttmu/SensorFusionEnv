import numpy as np
import math


from scipy.interpolate import griddata

class ReadLidar:

    def __init__(self):
        
        self.init_flg = 1
        self.read_flg = 0
        self.counter  = 0

        self.dataPacket = np.zeros(1218, 'uint8')
        self.dataPacket1 = np.zeros(1218)


# -------------------------------------------------------- #
    def dataPacketOut(self,d):

        if len(d) == 1218:
        
            self.dataPacket = [d[i] for i in range(1218)]
            self.dataPacket1 = [d[i] for i in range(1218)]
           
    
        elif len(d) == 1:
            self.dataPacket[1217] = d[0]
        #print( self.dataPacket1)
        if     self.dataPacket[   0] == 255 and self.dataPacket[   1] == 238 and self.dataPacket[ 100] == 255 and self.dataPacket[ 101] == 238 \
           and self.dataPacket[ 200] == 255 and self.dataPacket[ 201] == 238 and self.dataPacket[ 300] == 255 and self.dataPacket[ 301] == 238 \
           and self.dataPacket[ 400] == 255 and self.dataPacket[ 401] == 238 and self.dataPacket[ 500] == 255 and self.dataPacket[ 501] == 238 \
           and self.dataPacket[ 600] == 255 and self.dataPacket[ 601] == 238 and self.dataPacket[ 700] == 255 and self.dataPacket[ 701] == 238 \
           and self.dataPacket[ 800] == 255 and self.dataPacket[ 801] == 238 and self.dataPacket[ 900] == 255 and self.dataPacket[ 901] == 238 \
           and self.dataPacket[1000] == 255 and self.dataPacket[1001] == 238 and self.dataPacket[1100] == 255 and self.dataPacket[1101] == 238:

            self.init_flg = 0
            self.read_flg = 1

        if self.init_flg == 1:
            self.dataPacket[0:1217] = self.dataPacket[1:1218]

#
        return self.read_flg, self.dataPacket

# -------------------------------------------------------- #
    def read_timestamp(self,dataPacket):
        
        timestamp =   dataPacket[1210] *pow(16, 0) \
                    + dataPacket[1211] *pow(16, 2) \
                    + dataPacket[1212] *pow(16, 4) \
                    + dataPacket[1213] *pow(16, 6) \
                    + dataPacket[1214] *pow(16, 8) \
                    + dataPacket[1215] *pow(16,10) \
                    + dataPacket[1216] *pow(16,12) \
                    + dataPacket[1217] *pow(16,14)
        # timestamp =   dataPacket[1200] *pow(16, 0) \
        #             + dataPacket[1201] *pow(16, 2) \
        #             + dataPacket[1202] *pow(16, 4) \
        #             + dataPacket[1203] *pow(16, 6) \
                    
        
        return timestamp

# -------------------------------------------------------- #
    def data2spherical(self,data):
        pos_spherical = np.zeros((17,24), 'uint16' )

        for i in range(17):
            for j in range(0,24,2):
                if i == 0:
                    pos_spherical[0][j] = data[50*j+2] + data[50*j+3]*256

                else:
                    pos_spherical[i][j] = data[50*j+3*i+1] + data[50*j+3*i+2]*256


            for j in range(1,24,2):
                if i == 0:
                    if j != 23:
                        if pos_spherical[0][j-1] < pos_spherical[0][j+1]:
                            pos_spherical[0][j] = pos_spherical[0][j-1] + (pos_spherical[0][j+1]-pos_spherical[0][j-1])/2
                        else:
                            pos_spherical[0][j] = pos_spherical[0][j-1] + (pos_spherical[0][j+1]+36000-pos_spherical[0][j-1])/2
                            if pos_spherical[0][j] >= 36000:
                                pos_spherical[0][j] -= 36000
                    else:
                        if pos_spherical[0][22] > pos_spherical[0][21]:
                            pos_spherical[0][23] = pos_spherical[0][22] + (pos_spherical[0][22]-pos_spherical[0][21])
                            if pos_spherical[0][23] >= 36000:
                                pos_spherical[0][23] -= 36000
                        else:
                            pos_spherical[0][23] = pos_spherical[0][22] + (pos_spherical[0][22]+36000-pos_spherical[0][21])
                            if pos_spherical[0][23] >= 36000:
                                pos_spherical[0][23] -= 36000

                else:
                    pos_spherical[i][j] = data[50*j+3*i-1] + data[50*j+3*i]*256

        return pos_spherical

# -------------------------------------------------------- #
    def spherical2xyz(self,pos_spherical,ang_range):

        pos_xyz = np.zeros((1,3))
        omega_ref = [ -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15 ]
        
        k = 0
        pos_idx = []
        pos_xyz = []
        
        for i in range(pos_spherical.shape[1]):
            if pos_spherical[0][i] > 18000:
                pos_spherical[0][i] -= 36000

            if pos_spherical[0][i] < ang_range[0]*100:
                pass
            elif pos_spherical[0][i] > ang_range[1]*100:
                pass
            else:
                for j in range(1,pos_spherical.shape[0]):
                    if pos_spherical[j][i] == 0:
                        pass
                    else:
                        alpha = pos_spherical[0][i]/100 * math.pi/180
                        
                        omega = omega_ref[j-1]*2/3 * math.pi/180
                        pos_x = pos_spherical[j][i]*0.002 * math.cos(omega) * math.sin(alpha)
                        pos_y = pos_spherical[j][i]*0.002 * math.cos(omega) * math.cos(alpha)
                        pos_z = pos_spherical[j][i]*0.002 * math.sin(omega)

#                        if k == 0:
#                            pos_xyz = [[pos_x,pos_y,pos_z]]
#                            pos_idx = [j]
#                        else:
#                            pos_xyz = np.append(pos_xyz,[[pos_x,pos_y,pos_z]],axis=0)
#                            pos_idx = np.append(pos_idx,j)


                        pos_idx.append(j)
                        pos_xyz.append([pos_x,pos_y,pos_z])

                        k = k+1
            
        pos_idx = np.array(pos_idx)
        pos_xyz = np.array(pos_xyz)

        return pos_xyz, pos_idx

# -------------------------------------------------------- #
    def xyz2height(self, pos, a, b, c, d ):
        points_ = np.ones((len(pos),4))
        points_[:,:3] = pos[:,:]

        mat_pnts = np.mat(points_)

        vec = np.array([a,b,c])
        norm_vec = np.linalg.norm(vec)

        a /= norm_vec
        b /= norm_vec
        c /= norm_vec
        d /= norm_vec

        
        mat_abcd = np.mat([[a],[b],[c],[d]])
        dist = mat_pnts*mat_abcd
        dist = np.array(dist)

        return dist

# -------------------------------------------------------- #
    def interp_height(self, pnts_xyz, pnts_height):

        lidar_height = 1.5

        height_max =  0.15
        height_min = -0.15
        grid_x, grid_y = np.mgrid[1:15:31j,-10:10:101j]          # [vertical_ang, y] 

        index = np.where( (pnts_height>=height_min) & (pnts_height<=height_max) )[0]
        
        x = pnts_xyz[index,0]
        y = pnts_xyz[index,1]
        z = pnts_height[index,0]
        
        grid_x = lidar_height/np.tan(grid_x*math.pi/180)
        index = np.where(grid_x[:,0]<=60)

        grid_x = grid_x[index,:]
        grid_y = grid_y[index,:]
        z_interp = griddata(np.vstack((x, y)).T, z, (grid_x, grid_y), method='linear')

        x = grid_x.reshape(-1,)
        y = grid_y.reshape(-1,)
        z = z_interp.reshape(-1,)

        index = np.where(~np.isnan(z))[0]
        x = x[index]
        y = y[index]
        z = z[index]

        xyz = np.vstack((x,y))
        xyz = np.vstack((xyz,z))

        return xyz.T


#        fig = pyplot.figure()
#        ax = Axes3D(fig)
#        ax.set_xlabel("X-axis")
#        ax.set_ylabel("Y-axis")
#        ax.set_zlabel("Z-axis")
#        ax.plot(x, y, z, "o", color="#cccccc", ms=4, mew=0.5)
#        pyplot.show()
    

# -------------------------------------------------------- #
    def height2xyz(self, xyh, a, b, c, d):
        
        x = xyh[:,0]
        y = xyh[:,1]
        h = xyh[:,2]

        mat_xyh = np.ones((3,len(x)))
        mat_xyh[0,:] = x
        mat_xyh[1,:] = y
        mat_xyh = np.mat(mat_xyh)

        vec = np.array([a,b,c])
        norm_vec = np.linalg.norm(vec)

        a /= norm_vec
        b /= norm_vec
        c /= norm_vec
        d /= norm_vec
        
        mat_abd = np.mat([a,b,d])

        z = (h-mat_abd*mat_xyh)/c
        z = np.array(z)

        xyz = xyh.T
        xyz[2,:] = z
        xyz = xyz.T

        return xyz


