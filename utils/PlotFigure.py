import matplotlib.pyplot as plt
import matplotlib.animation as animation
from ctypes import *

import cv2
import numpy as np
import math
import copy

import sys

import inspect

import time

#from optimize3D import Optimize3D

Coef = 7#900/80
LON_OFFSET = 950
LAT_OFFSET = 150

# ================================================================= #
class PlotFigure:
    def __init__(self, args, Version=0, Video=0, FileName='output.avi', Show=1, ConstYawMode=1):
        fig = plt.figure()
        self.fig_ax = fig.add_subplot(111,projection='3d')
        self.scatter_plot = self.fig_ax.scatter(0,0,0)
        self.version = Version
        self.video = Video
        self.show = Show
        self.constYawMode = ConstYawMode
        #self.rot3d = Optimize3D()
        
        #self.constYaw = -0.9 # naname45
        self.constYaw = -1.575 # mayoko

        self.lat = 0
        self.lon = 0

        if self.version == 0:
            plotEPM4 = np.zeros((1000,1800,3),dtype=np.uint8)

            plotText = np.full((1000,300,3),0,dtype=np.uint8)
            plotEPM4[0:1000, 1500:1800] = plotText

            plotREM = np.full((700,300,3),0,dtype=np.uint8)
            plotEPM4[300:1000, 0:300] = plotREM

            
            self.latSize = 20480
            self.lonSize = 20480
            
            self.GPScoef = 90000
            self.XYcoef = 10#27#10#6#14#20 #[pixel/m]
            self.GPSfft = True

        if self.video == 1:
            FileName = args.data_root + args.data_name + "/result.avi"
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.VideoWriter = cv2.VideoWriter(FileName,fourcc,36,(plotEPM4.shape[1],plotEPM4.shape[0]))

        self.plotEPM4 = plotEPM4

# ================================================================================== #
    def get_BaseImage(self):
        img = self.plotEPM4.copy()
        return img     

# ================================================================================== #
    def showImage(self,figure):
        if self.video == 1:
            self.VideoWriter.write(figure)
        if self.show == 1:
            cv2.imshow('window',figure)
            cv2.waitKey(1)
            
# ================================================================================== #
    def saveImage(self,figure,FILENAME = 'map.png'):
        cv2.imwrite(FILENAME,figure)


# ================================================================================== #
    def PlotFigure_Movie(self,frame,Base = np.zeros(2)):
        if self.version == 0:
            self.lon_min = int(self.lonSize/2 - self.lon-490)
            self.lon_max = int(self.lonSize/2 - self.lon+490)
            self.lat_min = int(self.latSize/2 + self.lat-890+300)
            self.lat_max = int(self.latSize/2 + self.lat+890-300)
            if ( 0 <= self.lon_min ) & ( self.lon_max < self.lonSize):
                self.lon_from = self.lon_min
                self.lon_to   = self.lon_max
            if ( 0 <= self.lat_min ) & ( self.lat_max < self.latSize):
                self.lat_from = self.lat_min
                self.lat_to   = self.lat_max
            frame2 = frame[self.lon_from:self.lon_to,self.lat_from:self.lat_to,:]
            
            Base[ 10: 990, 10+300:1790-300] = frame2

        return Base


    def PlotFigure_USBCam(self,frame,Base = np.zeros(2)):
        if self.version == 0:
            Base[ 0: 270, 0: 480] = frame

            cv2.line(Base, (1600,20),(1600+self.XYcoef,20), (255,255,255), 1)
            cv2.putText(Base, "1m", (1570,20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1, cv2.LINE_AA)
        
        return Base

    
    def get_line(depth=0):
        line = inspect.currentframe().f_back
        return line.f_lineno


    #############################################################################
    def PlotFigure_lidar(self, lidar, z_min, z_max, Base = np.zeros(2)):
        Img = Base

        LAT_OFFSET2 = LAT_OFFSET+750
        LON_OFFSET2 = LON_OFFSET-450
        TF = -self.constYaw

        ## Limit Z
        lidar_limited = self.lidar_limit_z(lidar, z_min, z_max)
        ## Pick up XY
        lidar_xy = lidar_limited[:,[0,1]]

        ##---------------  Conv. LiDAR Coordinate -> REM Coordinate  ------
        ## Longitudinal(tate houkou)      X       ->       Z 
        ##      Lateral(yoko houkou)     -Y       ->       X
        tmpz = lidar_limited[:,0]
        tmpx = -lidar_limited[:,1]
        lidar_zx = np.array([tmpx,tmpz])
        ##-----------------------------------------------------------------
        
        rot = np.array([[math.cos(TF),-math.sin(TF)],[math.sin(TF),math.cos(TF)]])
        lidar_ = np.dot(rot,lidar_zx)
        #print(lidar_zx.shape)
        for i in range(lidar_.shape[1]):
            Img = cv2.circle(Img,(int(lidar_[0,i]*self.XYcoef+LAT_OFFSET2),int(LON_OFFSET2-lidar_[1,i]*self.XYcoef)), 0, (160,160,160), -1)
            #Img = cv2.circle(Img,(int(lidar_[0,i]*self.XYcoef+LAT_OFFSET2),int(LON_OFFSET2-lidar_[1,i]*self.XYcoef)), 1, (85*intensity[i]*0.01,230*intensity[i]*0.01,230*intensity[i]*0.01), -1)

        return Img

    ###############
    def calibrate_lidar(self, lidar, lidar_calib_param, x_ofs, y_ofs, z_ofs):
        intensity = lidar[3,:]
        lidar = lidar[[0,1,2],:]
        lidar_ = self.rot3d.apply_Rot_and_TF(lidar_calib_param,lidar.T)
        lidar_ = self.rot3d.apply_TF(lidar_, x_ofs, y_ofs, z_ofs)
        return lidar_, intensity
    
    ###############
    def NearestPoint(self, Points):
        if len(Points) != 0:
            dist_list = np.linalg.norm(Points, axis=1)
            #print(dist_list)
            # print(Points)
            # print(dist_list)
            #nearest_point = dist_list.min()
            min_index = np.argmin(dist_list)
            nearest_point = Points[min_index]
            return nearest_point

    ###############
    def NearestLidarPoint(self, lidar, NearP):
        #///////// Calc diff. from Lidar
        LidarRadarX, LidarRadarY = 999, 999
        lidar = lidar[:,[0,1,2]]
        lidar_obj = self.lidar_limit(lidar, NearP[0], NearP[1])
        if len(lidar_obj) != 0:
            NearP_lidar = self.NearestPoint(lidar_obj)

            LidarRadarX = NearP_lidar[0]
            LidarRadarY = NearP_lidar[1]
        
        return LidarRadarX, LidarRadarY
        #\\\\\\\\\


    def lidar_limit_z(self, lidar, a, b):
        ## lidar shape [||||]
        lidar_ = lidar[ (a<lidar[:,2])&(lidar[:,2]<b) ]
        return lidar_

    def lidar_limit(self, lidar, x, y):
        ## lidar shape [||||]
        a = 2
        b = 1
        c = 1
        lidar_ = lidar[ (x-a<lidar[:,0])&(lidar[:,0]<x+a)& \
                        (y-b<lidar[:,1])&(lidar[:,1]<y+b)& \
                        (-c <lidar[:,2])&(lidar[:,2]< c) ]
        return lidar_

    def TF_to_REM(self, x, y):
        x = -y
        z = x
        return x, z

    def Rotation(self,x,y,theta):
        cosTF,sinTF = math.cos(theta), math.sin(theta)
        x_ = x*cosTF - y*sinTF
        y_ = x*sinTF + y*cosTF
        return x_, y_

    def TransCoordinateNormal2pic(self,x,y):
        LAT_OFFSET2 = LAT_OFFSET+750
        LON_OFFSET2 = LON_OFFSET-450
        X = -y
        Z = x
        TF = -self.constYaw
        cosTF,sinTF = math.cos(TF), math.sin(TF)
        X_ = X*cosTF - Z*sinTF
        Z_ = X*sinTF + Z*cosTF
        pic_x = int(X_*self.XYcoef+LAT_OFFSET2)
        pic_z = int(LON_OFFSET2-Z_*self.XYcoef)
        return pic_x, pic_z

    ############################################################################
    def PlotFigure_ARS510_Obj(self, ARS510_obj, lidar, Base = np.zeros(2)):
        Img = Base

        lidarOBJ = np.zeros((40,2))
        NearP = np.zeros((40,5))

        LAT_OFFSET2 = LAT_OFFSET+750
        LON_OFFSET2 = LON_OFFSET-450
        TF = -self.constYaw
        
        ## Objects
        #print(ARS510_obj[0].ui_num_of_used_object)
        for i in range(len(ARS510_obj)):
            category = ARS510_obj[i].e_classification
            if category == 1:
            #if category == 0 or category == 1 or category == 2 or category == 3 or category == 4 or category == 5 or category == 6 or category == 7:
                DistX = ARS510_obj[i].f_dist_x
                DistY = ARS510_obj[i].f_dist_y
                width = ARS510_obj[i].f_width
                length = ARS510_obj[i].f_length
                orientation = ARS510_obj[i].f_orientation
                #print(DistX)

                # Four corner's points
                p0_x,p0_y,p1_x,p1_y,p2_x,p2_y,p3_x,p3_y = self.Rot_rectangle_ARS510(DistX, DistY, width, length, orientation)

                # to picture coordinate
                pic_x0,pic_y0 = self.TransCoordinateNormal2pic(p0_x, p0_y)
                pic_x1,pic_y1 = self.TransCoordinateNormal2pic(p1_x, p1_y)
                pic_x2,pic_y2 = self.TransCoordinateNormal2pic(p2_x, p2_y)
                pic_x3,pic_y3 = self.TransCoordinateNormal2pic(p3_x, p3_y)

                color = (15,230,240)# Yellow
                Img = cv2.line(Img, (pic_x0,pic_y0), (pic_x1,pic_y1), color,thickness=1)
                Img = cv2.line(Img, (pic_x1,pic_y1), (pic_x2,pic_y2), color,thickness=1)
                Img = cv2.line(Img, (pic_x2,pic_y2), (pic_x3,pic_y3), color,thickness=1)
                Img = cv2.line(Img, (pic_x3,pic_y3), (pic_x0,pic_y0), color,thickness=2)
                #Img = cv2.putText(Img,str(ARS510_obj[i].u_id), (pic_x3,pic_y3), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 1, cv2.LINE_AA)
                Img = cv2.putText(Img,str(round(ARS510_obj[i].f_probability_of_existence,2)), (pic_x3,pic_y3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
                Img = cv2.putText(Img,str(ARS510_obj[i].ui_life_cycles), (pic_x3,pic_y3+14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

                ## Calc. LiDAR's point
                # Calc. Nearest Point
                points = np.array([[p0_x,p0_y],[p1_x,p1_y],[p2_x,p2_y],[p3_x,p3_y]])
                nearest_point = self.NearestPoint(points)
                #print(nearest_point)
                # Calc. Lidar point
                NearP_lidarX, NearP_lidarY = self.NearestLidarPoint(lidar,nearest_point)
                # to picture coordinate
                pic_x_lidar,pic_y_lidar = self.TransCoordinateNormal2pic(NearP_lidarX, NearP_lidarY)
                pic_x_radar,pic_y_radar = self.TransCoordinateNormal2pic(nearest_point[0], nearest_point[1])
                Img = cv2.circle(Img, (pic_x_lidar,pic_y_lidar), 3, (100,180,240), -1)
                Img = cv2.circle(Img, (pic_x_radar,pic_y_radar), 2, (255,255,255), -1)

                # pic_XX,pic_YY = self.TransCoordinateNormal2pic(DistX,DistY)
                # Img = cv2.circle(Img, (pic_XX,pic_YY), 3, color, -1)

                if i < 40:
                    lidarOBJ[i,0] = NearP_lidarX
                    lidarOBJ[i,1] = NearP_lidarY
                    NearP[i,0] = ARS510_obj[i].u_id
                    NearP[i,1] = nearest_point[0]
                    NearP[i,2] = nearest_point[1]
                    NearP[i,3] = ARS510_obj[i].a_vrel_x
                    NearP[i,4] = ARS510_obj[i].a_vrel_y
                else:
                    temp_lidarOBJ = np.zeros((1,2))
                    temp_lidarOBJ[0,0] = NearP_lidarX
                    temp_lidarOBJ[0,1] = NearP_lidarY
                    np.append(lidarOBJ, temp_lidarOBJ, axis=0)
                    
                    temp_NearP = np.zeros((1,5))
                    temp_NearP[0,0] = ARS510_obj[i].u_id
                    temp_NearP[0,1] = nearest_point[0]
                    temp_NearP[0,2] = nearest_point[1]
                    temp_NearP[0,3] = ARS510_obj[i].a_vrel_x
                    temp_NearP[0,4] = ARS510_obj[i].a_vrel_y
                    np.append(NearP, temp_NearP, axis=0)

        return Img, lidarOBJ, NearP

    def Rot_rectangle_ARS510(self, X, Y, width, length, orientation):
        p0_x = X + 0*math.cos(orientation) - (-width/2)*math.sin(orientation)
        p0_y = Y + 0*math.sin(orientation) + (-width/2)*math.cos(orientation)
        p1_x = X + length*math.cos(orientation) - (-width/2)*math.sin(orientation)
        p1_y = Y + length*math.sin(orientation) + (-width/2)*math.cos(orientation)
        p2_x = X + length*math.cos(orientation) - (width/2)*math.sin(orientation)
        p2_y = Y + length*math.sin(orientation) + (width/2)*math.cos(orientation)
        p3_x = X + 0*math.cos(orientation) - (width/2)*math.sin(orientation)
        p3_y = Y + 0*math.sin(orientation) + (width/2)*math.cos(orientation)

        # p0_x = X + 0*math.cos(orientation) - width/2*math.sin(orientation)
        # p0_y = Y + 0*math.sin(orientation) + width/2*math.cos(orientation)
        # p1_x = X + (-length)*math.cos(orientation) - width/2*math.sin(orientation)
        # p1_y = Y + (-length)*math.sin(orientation) + width/2*math.cos(orientation)
        # p2_x = X + (-length)*math.cos(orientation) - (-width/2)*math.sin(orientation)
        # p2_y = Y + (-length)*math.sin(orientation) + (-width/2)*math.cos(orientation)
        # p3_x = X + 0*math.cos(orientation) - (-width/2)*math.sin(orientation)
        # p3_y = Y + 0*math.sin(orientation) + (-width/2)*math.cos(orientation)

        # p0_x = X + 0*math.cos(orientation) - (-width/2)*math.sin(orientation)
        # p0_y = Y + 0*math.sin(orientation) + (-width/2)*math.cos(orientation)
        # p1_x = X - length*math.cos(orientation) - (-width/2)*math.sin(orientation)
        # p1_y = Y - length*math.sin(orientation) + (-width/2)*math.cos(orientation)
        # p2_x = X - length*math.cos(orientation) - (width/2)*math.sin(orientation)
        # p2_y = Y - length*math.sin(orientation) + (width/2)*math.cos(orientation)
        # p3_x = X + 0*math.cos(orientation) - (width/2)*math.sin(orientation)
        # p3_y = Y + 0*math.sin(orientation) + (width/2)*math.cos(orientation)
        return p0_x,p0_y,p1_x,p1_y,p2_x,p2_y,p3_x,p3_y

    ############################################################################
    def PlotFigure_SRR520_Obj(self, SRR520_obj, witchRadar, lidar, Base = np.zeros(2)):
        Img = Base

        lidarOBJ = np.zeros((40,2))
        NearP = np.zeros((40,5))

        LAT_OFFSET2 = LAT_OFFSET+750
        LON_OFFSET2 = LON_OFFSET-450
        TF = -self.constYaw
        
        ## Objects
        for i in range(len(SRR520_obj)):
            category = SRR520_obj[i].e_classification
            if category == 1:
            #if category == 0 or category == 1 or category == 2 or category == 3 or category == 4 or category == 5 or category == 6 or category == 7:
                DistX = SRR520_obj[i].f_dist_x
                DistY = SRR520_obj[i].f_dist_y
                width_left = SRR520_obj[i].f_width_left
                width_right = SRR520_obj[i].f_width_right
                length_front = SRR520_obj[i].f_length_front
                length_rear = SRR520_obj[i].f_length_rear
                orientation = SRR520_obj[i].f_orientation

                # Four corner's points
                p0_x,p0_y,p1_x,p1_y,p2_x,p2_y,p3_x,p3_y = self.Rot_rectangle_SRR520(DistX, DistY, width_left, width_right, length_front, length_rear, orientation)

                # to picture coordinate
                pic_x0,pic_y0 = self.TransCoordinateNormal2pic(p0_x, p0_y)
                pic_x1,pic_y1 = self.TransCoordinateNormal2pic(p1_x, p1_y)
                pic_x2,pic_y2 = self.TransCoordinateNormal2pic(p2_x, p2_y)
                pic_x3,pic_y3 = self.TransCoordinateNormal2pic(p3_x, p3_y)

                if witchRadar == 'FL': color = (10,255,40)# Green
                if witchRadar == 'FR': color = (255,230,90)# Blue
                if witchRadar == 'RL': color = (15,120,235)# Orange
                if witchRadar == 'RR': color = (200,85,240)# Purple
                Img = cv2.line(Img, (pic_x0,pic_y0), (pic_x1,pic_y1), color,thickness=1)
                Img = cv2.line(Img, (pic_x1,pic_y1), (pic_x2,pic_y2), color,thickness=1)
                Img = cv2.line(Img, (pic_x2,pic_y2), (pic_x3,pic_y3), color,thickness=1)
                Img = cv2.line(Img, (pic_x3,pic_y3), (pic_x0,pic_y0), color,thickness=2)
                #Img = cv2.putText(Img,str(SRR520_obj[i].u_id), (pic_x3,pic_y3), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 1, cv2.LINE_AA)
                Img = cv2.putText(Img,str(round(SRR520_obj[i].f_probability_of_existence,2)), (pic_x3,pic_y3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
                Img = cv2.putText(Img,str(SRR520_obj[i].ui_life_cycles), (pic_x3,pic_y3+14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

                ## Calc. LiDAR's point
                # Calc. Nearest Point
                points = np.array([[p0_x,p0_y],[p1_x,p1_y],[p2_x,p2_y],[p3_x,p3_y]])
                nearest_point = self.NearestPoint(points)
                # Calc. Lidar point
                NearP_lidarX, NearP_lidarY = self.NearestLidarPoint(lidar,nearest_point)
                # to picture coordinate
                pic_x_lidar,pic_y_lidar = self.TransCoordinateNormal2pic(NearP_lidarX, NearP_lidarY)
                pic_x_radar,pic_y_radar = self.TransCoordinateNormal2pic(nearest_point[0], nearest_point[1])
                Img = cv2.circle(Img, (pic_x_lidar,pic_y_lidar), 3, (100,180,240), -1)
                Img = cv2.circle(Img, (pic_x_radar,pic_y_radar), 2, (255,255,255), -1)

                # pic_XX,pic_YY = self.TransCoordinateNormal2pic(DistX,DistY)
                # Img = cv2.circle(Img, (pic_XX,pic_YY), 3, color, -1)

                if i < 40:
                    lidarOBJ[i,0] = NearP_lidarX
                    lidarOBJ[i,1] = NearP_lidarY
                    NearP[i,0] = SRR520_obj[i].u_id
                    NearP[i,1] = nearest_point[0]
                    NearP[i,2] = nearest_point[1]
                    NearP[i,3] = SRR520_obj[i].a_vrel_x
                    NearP[i,4] = SRR520_obj[i].a_vrel_y
                else:
                    temp_lidarOBJ = np.zeros((1,2))
                    temp_lidarOBJ[0,0] = NearP_lidarX
                    temp_lidarOBJ[0,1] = NearP_lidarY
                    np.append(lidarOBJ, temp_lidarOBJ, axis=0)
                    
                    temp_NearP = np.zeros((1,5))
                    temp_NearP[0,0] = SRR520_obj[i].u_id
                    temp_NearP[0,1] = nearest_point[0]
                    temp_NearP[0,2] = nearest_point[1]
                    temp_NearP[0,3] = SRR520_obj[i].a_vrel_x
                    temp_NearP[0,4] = SRR520_obj[i].a_vrel_y
                    np.append(NearP, temp_NearP, axis=0)

        return Img, lidarOBJ, NearP

    def Rot_rectangle_SRR520(self, X, Y, w_left, w_right, l_front, l_rear, orientation):
        # p0_x = X + (-l_front)*math.cos(orientation) - (-w_left)*math.sin(orientation)
        # p0_y = Y + (-l_front)*math.sin(orientation) + (-w_left)*math.cos(orientation)
        # p1_x = X + l_rear*math.cos(orientation) - (-w_left)*math.sin(orientation)
        # p1_y = Y + l_rear*math.sin(orientation) + (-w_left)*math.cos(orientation)
        # p2_x = X + l_rear*math.cos(orientation) - w_right*math.sin(orientation)
        # p2_y = Y + l_rear*math.sin(orientation) + w_right*math.cos(orientation)
        # p3_x = X + (-l_front)*math.cos(orientation) - w_right*math.sin(orientation)
        # p3_y = Y + (-l_front)*math.sin(orientation) + w_right*math.cos(orientation)

        # p0_x = X + l_front*math.cos(orientation) - w_right*math.sin(orientation)
        # p0_y = Y + l_front*math.sin(orientation) + w_right*math.cos(orientation)
        # p1_x = X + (-l_rear)*math.cos(orientation) - w_right*math.sin(orientation)
        # p1_y = Y + (-l_rear)*math.sin(orientation) + w_right*math.cos(orientation)
        # p2_x = X + (-l_rear)*math.cos(orientation) - (-w_left)*math.sin(orientation)
        # p2_y = Y + (-l_rear)*math.sin(orientation) + (-w_left)*math.cos(orientation)
        # p3_x = X + l_front*math.cos(orientation) - (-w_left)*math.sin(orientation)
        # p3_y = Y + l_front*math.sin(orientation) + (-w_left)*math.cos(orientation)

        p0_x = X + (l_front)*math.cos(orientation) - (-w_right)*math.sin(orientation)
        p0_y = Y + (l_front)*math.sin(orientation) + (-w_right)*math.cos(orientation)
        p1_x = X - l_rear*math.cos(orientation) - (-w_right)*math.sin(orientation)
        p1_y = Y - l_rear*math.sin(orientation) + (-w_right)*math.cos(orientation)
        p2_x = X - l_rear*math.cos(orientation) - w_left*math.sin(orientation)
        p2_y = Y - l_rear*math.sin(orientation) + w_left*math.cos(orientation)
        p3_x = X + (l_front)*math.cos(orientation) - w_left*math.sin(orientation)
        p3_y = Y + (l_front)*math.sin(orientation) + w_left*math.cos(orientation)
        return p0_x,p0_y,p1_x,p1_y,p2_x,p2_y,p3_x,p3_y

    ############################################################################
    def PlotFigure_Radar_Obj(self, Radar, Base = np.zeros(2)):
        Img = Base

        LAT_OFFSET2 = LAT_OFFSET+750
        LON_OFFSET2 = LON_OFFSET-450
        TF = -self.constYaw
        
        ## Objects
        for i in range(len(Radar.Obj)):
            DistX = Radar.Obj[i].x
            DistY = Radar.Obj[i].y
            width_left = Radar.Obj[i].width_left
            width_right = Radar.Obj[i].width_right
            length_front = Radar.Obj[i].length_front
            length_rear = Radar.Obj[i].length_rear
            orientation = Radar.Obj[i].orientation

            # Four corner's points
            p0_x,p0_y,p1_x,p1_y,p2_x,p2_y,p3_x,p3_y = self.Rot_rectangle_SRR520(DistX, DistY, width_left, width_right, length_front, length_rear, orientation)

            # to picture coordinate
            pic_x0,pic_y0 = self.TransCoordinateNormal2pic(p0_x, p0_y)
            pic_x1,pic_y1 = self.TransCoordinateNormal2pic(p1_x, p1_y)
            pic_x2,pic_y2 = self.TransCoordinateNormal2pic(p2_x, p2_y)
            pic_x3,pic_y3 = self.TransCoordinateNormal2pic(p3_x, p3_y)

            color = (0,0,255)# Red
            Img = cv2.line(Img, (pic_x0,pic_y0), (pic_x1,pic_y1), color,thickness=1)
            Img = cv2.line(Img, (pic_x1,pic_y1), (pic_x2,pic_y2), color,thickness=1)
            Img = cv2.line(Img, (pic_x2,pic_y2), (pic_x3,pic_y3), color,thickness=1)
            Img = cv2.line(Img, (pic_x3,pic_y3), (pic_x0,pic_y0), color,thickness=1)
            #Img = cv2.putText(Img,str(SRR520_obj[i].u_id), (pic_x3,pic_y3), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 1, cv2.LINE_AA)
            
        return Img

    def PlotFigure_Radar_Obj_2(self, Radar, Base = np.zeros(2)):
        Img = Base

        LAT_OFFSET2 = LAT_OFFSET+750
        LON_OFFSET2 = LON_OFFSET-450
        TF = -self.constYaw
        
        ## Objects
        for i in range(len(Radar.obj)):
            category = Radar.obj[i].e_classification
            if category == 1:
                DistX = Radar.obj[i].x
                DistY = Radar.obj[i].y
                width_left = Radar.obj[i].width_left
                width_right = Radar.obj[i].width_right
                length_front = Radar.obj[i].length_front
                length_rear = Radar.obj[i].length_rear
                orientation = Radar.obj[i].orientation

                # Four corner's points
                p0_x,p0_y,p1_x,p1_y,p2_x,p2_y,p3_x,p3_y = self.Rot_rectangle_SRR520(DistX, DistY, width_left, width_right, length_front, length_rear, orientation)

                # to picture coordinate
                pic_x0,pic_y0 = self.TransCoordinateNormal2pic(p0_x, p0_y)
                pic_x1,pic_y1 = self.TransCoordinateNormal2pic(p1_x, p1_y)
                pic_x2,pic_y2 = self.TransCoordinateNormal2pic(p2_x, p2_y)
                pic_x3,pic_y3 = self.TransCoordinateNormal2pic(p3_x, p3_y)

                color = (0,0,255)# Red
                Img = cv2.line(Img, (pic_x0,pic_y0), (pic_x1,pic_y1), color,thickness=1)
                Img = cv2.line(Img, (pic_x1,pic_y1), (pic_x2,pic_y2), color,thickness=1)
                Img = cv2.line(Img, (pic_x2,pic_y2), (pic_x3,pic_y3), color,thickness=1)
                Img = cv2.line(Img, (pic_x3,pic_y3), (pic_x0,pic_y0), color,thickness=1)
                #Img = cv2.putText(Img,str(SRR520_obj[i].u_id), (pic_x3,pic_y3), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 1, cv2.LINE_AA)
            
        return Img

    ############################################################################
    def PlotFigure_ARS510_Csr(self, ARS510_csr, Base = np.zeros(2)):
        Img = Base

        LAT_OFFSET2 = LAT_OFFSET+750
        LON_OFFSET2 = LON_OFFSET-450
        TF = -self.constYaw

        snsPos_x = 0.882
        snsPos_y = 0.0
        
        ## Clusters
        for i in range(len(ARS510_csr)):
            radial_dist = ARS510_csr[i].f_RangeRad
            Azimuth_angle_0 = ARS510_csr[i].a_AzAng_0
            Azimuth_angle_1 = ARS510_csr[i].a_AzAng_1

            if np.isnan(radial_dist) or np.isnan(Azimuth_angle_0) or np.isnan(Azimuth_angle_1):
                break

            x = radial_dist*math.cos(Azimuth_angle_0) + snsPos_x
            y = radial_dist*math.sin(Azimuth_angle_0) + snsPos_y

            # to picture coordinate
            pic_x,pic_y = self.TransCoordinateNormal2pic(x, y)

            color = (15,230,240)# Yellow
            if 0 < pic_x and pic_x < 1800 and 0 < pic_y and pic_y < 1000:
                Img = cv2.circle(Img, (pic_x,pic_y), 1, color, -1)
 
        return Img

    ############################################################################
    def PlotFigure_SRR520_Csr(self, SRR520_csr, witchRadar, Base = np.zeros(2)):
        Img = Base

        LAT_OFFSET2 = LAT_OFFSET+750
        LON_OFFSET2 = LON_OFFSET-450
        TF = -self.constYaw

        if witchRadar == 'FL':
            snsPos_x = 0.643
            snsPos_y = 0.809
        if witchRadar == 'FR':
            snsPos_x = 0.643
            snsPos_y = -0.809
        if witchRadar == 'RL':
            snsPos_x = -3.348
            snsPos_y = 0.695
        if witchRadar == 'RR':
            snsPos_x = -3.348
            snsPos_y = -0.695

        ## Clusters
        #print(SRR520_csr[0].ui_num_of_clusters)
        for i in range(len(SRR520_csr)):
            radial_dist = SRR520_csr[i].f_RangeRad
            Azimuth_angle = SRR520_csr[i].a_AzAng_0

            x = radial_dist*math.cos(Azimuth_angle) + snsPos_x
            y = radial_dist*math.sin(Azimuth_angle) + snsPos_y

            # to picture coordinate
            pic_x,pic_y = self.TransCoordinateNormal2pic(x, y)

            if witchRadar == 'FL': color = (10,255,40)# Green
            if witchRadar == 'FR': color = (255,230,90)# Blue
            if witchRadar == 'RL': color = (15,120,235)# Orange
            if witchRadar == 'RR': color = (200,85,240)# Purple
            if 0 < pic_x and pic_x < 1800 and 0 < pic_y and pic_y < 1000:
                if SRR520_csr[i].e_scan_type == 0:
                    Img = cv2.circle(Img, (pic_x,pic_y), 3, color)
                else:
                    Img = cv2.circle(Img, (pic_x,pic_y), 1, color, -1)
 
        return Img


    def PlotFigure_arrow(self, Base = np.zeros(2)):
        Img = Base

        LAT_OFFSET2 = LAT_OFFSET+750
        LON_OFFSET2 = LON_OFFSET-450
        yaw = -self.constYaw
        x = 0
        z = 3
        x_ = x*math.cos(yaw)-z*math.sin(yaw)
        z_ = x*math.sin(yaw)+z*math.cos(yaw)
        pic_x = int(x_*self.XYcoef+LAT_OFFSET2)
        pic_z = int(LON_OFFSET2-z_*self.XYcoef)
        cv2.arrowedLine(Img, (900,500), (pic_x,pic_z), (255,255,255), thickness=2)

        return Img


    def PlotFigure_scale(self, Base = np.zeros(2)):
        Img = Base

        LAT_OFFSET2 = LAT_OFFSET+750
        LON_OFFSET2 = LON_OFFSET-450

        #Img[0:1000, (LAT_OFFSET2+0*self.XYcoef), 2] = 255
        Img[0:1000, (LAT_OFFSET2+0*self.XYcoef), 2] = 255

        for i in range(2):
            sign = 1
            if i == 0: sign = -1
            Img[0:1000, (LAT_OFFSET2+(5*sign)*self.XYcoef), 2] = 100
            Img[0:1000, (LAT_OFFSET2+(10*sign)*self.XYcoef), 2] = 255
            Img[0:1000, (LAT_OFFSET2+(15*sign)*self.XYcoef), 2] = 100
            Img[0:1000, (LAT_OFFSET2+(20*sign)*self.XYcoef), 2] = 255
            Img[0:1000, (LAT_OFFSET2+(25*sign)*self.XYcoef), 2] = 100
            Img[0:1000, (LAT_OFFSET2+(30*sign)*self.XYcoef), 2] = 255
            Img[0:1000, (LAT_OFFSET2+(35*sign)*self.XYcoef), 2] = 100
            Img[0:1000, (LAT_OFFSET2+(40*sign)*self.XYcoef), 2] = 255
            Img[0:1000, (LAT_OFFSET2+(45*sign)*self.XYcoef), 2] = 100
            Img[0:1000, (LAT_OFFSET2+(50*sign)*self.XYcoef), 2] = 255
            Img[0:1000, (LAT_OFFSET2+(55*sign)*self.XYcoef), 2] = 100
            Img[0:1000, (LAT_OFFSET2+(60*sign)*self.XYcoef), 2] = 255
            Img[0:1000, (LAT_OFFSET2+(65*sign)*self.XYcoef), 2] = 100
            Img[0:1000, (LAT_OFFSET2+(70*sign)*self.XYcoef), 2] = 255
            Img[0:1000, (LAT_OFFSET2+(75*sign)*self.XYcoef), 2] = 100
            Img[0:1000, (LAT_OFFSET2+(80*sign)*self.XYcoef), 2] = 255
            Img[0:1000, (LAT_OFFSET2+(85*sign)*self.XYcoef), 2] = 100
            #Img[0:1000, (LAT_OFFSET2+(90*sign)*self.XYcoef), 2] = 255

        return Img

    def PlotFigure_t_xavier(self, t_xavier, Base = np.zeros(2)):
        Img = Base
        cv2.putText(Img, str(t_xavier), (500,15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1, cv2.LINE_AA)
        return Img

    def PlotFigure_CAN(self, can_v, Base = np.zeros(2)):
        Img = Base
        cv2.putText(Img, str(round(can_v/3.6,2))+"[m/s]", (500,15+14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1, cv2.LINE_AA)
        cv2.putText(Img, str(round(can_v,2))+"[km/h]", (500,15+28), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1, cv2.LINE_AA)
        return Img



    ##=====================================
    ## Draw Integrated Object
    ##=====================================
    def PlotFigure_integrated_obj(self, obj, Img, mode="normal"):
        LAT_OFFSET2 = LAT_OFFSET+750
        LON_OFFSET2 = LON_OFFSET-450
        TF = -self.constYaw
        
        ## Objects
        #print(ARS510_obj[0].ui_num_of_used_object)
        for i in range(len(obj.obj)):
            DistX = obj.obj[i].x_s[0]
            DistY = obj.obj[i].x_s[1]
            width_left = obj.obj[i].x_s[6]*0.5
            width_right = obj.obj[i].x_s[6]*0.5
            length_front = obj.obj[i].x_s[7]
            length_rear = 0
            orientation = obj.obj[i].x_s[8]

            # Four corner's points
            p0_x,p0_y,p1_x,p1_y,p2_x,p2_y,p3_x,p3_y = self.Rot_rectangle_SRR520(DistX, DistY, width_left, width_right, length_front, length_rear, orientation)

            # to picture coordinate
            pic_x0,pic_y0 = self.TransCoordinateNormal2pic(p0_x, p0_y)
            pic_x1,pic_y1 = self.TransCoordinateNormal2pic(p1_x, p1_y)
            pic_x2,pic_y2 = self.TransCoordinateNormal2pic(p2_x, p2_y)
            pic_x3,pic_y3 = self.TransCoordinateNormal2pic(p3_x, p3_y)

            color = (0,0,255)# Red
            Img = cv2.line(Img, (pic_x0,pic_y0), (pic_x1,pic_y1), color,thickness=2)
            Img = cv2.line(Img, (pic_x1,pic_y1), (pic_x2,pic_y2), color,thickness=2)
            Img = cv2.line(Img, (pic_x2,pic_y2), (pic_x3,pic_y3), color,thickness=2)
            Img = cv2.line(Img, (pic_x3,pic_y3), (pic_x0,pic_y0), color,thickness=1)
            # Img = cv2.putText(Img,str(obj.obj[i].ID), (pic_x3,pic_y3+14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
            # Img = cv2.putText(Img,str(obj.obj[i].age), (pic_x3,pic_y3+28), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
            # Img = cv2.putText(Img,str(obj.obj[i].unobserved_count), (pic_x3,pic_y3+42), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
            # Img = cv2.putText(Img,str(obj.obj[i].x_s), (pic_x3,pic_y3+56), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
            # Img = cv2.putText(Img,str(round(np.rad2deg(obj.obj[i].x_s[8]),2)), (pic_x3,pic_y3+28), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
            # Img = cv2.putText(Img,str(round(np.sqrt(obj.obj[i].x_s[2]**2+obj.obj[i].x_s[3]**2),2)), (pic_x3,pic_y3+42), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
            #Img = self.valiancePlot()

            pic_DistX,pic_DistY = self.TransCoordinateNormal2pic(DistX, DistY)
            Img = cv2.circle(Img, (int(pic_DistX),int(pic_DistY)), 3, color, -1)

        return Img



    ##=====================================
    ## Draw Integrated Observed Object
    ##=====================================
    def PlotFigure_integrated_obs_obj(self, obj, Img, color=(0,0,255), mode="Normal"):
        LAT_OFFSET2 = LAT_OFFSET+750
        LON_OFFSET2 = LON_OFFSET-450
        TF = -self.constYaw
        
        ## Objects
        for i in range(len(obj)):
            #DistX = obj[i].y[0]
            #DistY = obj[i].y[1]
            #width_left = obj[i].y[4]*0.5
            #width_right = obj[i].y[4]*0.5
            #length_front = obj[i].y[5]
            #length_rear = 0
            #orientation = obj[i].y[6]
            DistX = obj[i].x
            DistY = obj[i].y
            width_left = obj[i].w*0.5
            width_right = obj[i].w*0.5
            length_front = obj[i].l*0.5
            length_rear = obj[i].l*0.5
            orientation = obj[i].o
            #TODO 検出点の位置を再確認
            #if mode == "reference":
            #    #DistX = obj[i].x - obj[i].l*np.cos(-orientation)*0.5
            #    #DistY = obj[i].y + obj[i].l*np.sin(-orientation)*0.5 #特例
            #    width_left = obj[i].w*0.5
            #    width_right = obj[i].w*0.5
            #    length_front = obj[i].l*0.5
            #    length_rear = obj[i].l*0.5
            #    #orientation = -obj[i].o
            #elif mode == "svc220":
            #    length_front = obj[i].l * 0.5
            #    length_rear = obj[i].l * 0.5
            #    pass
            #elif mode == "ars510":
            #    #DistX = obj[i].x + obj[i].l*np.cos(orientation)*0.5
            #    #DistY = obj[i].y - obj[i].l*np.sin(orientation)*0.5
            #    width_left = obj[i].w*0.5
            #    width_right = obj[i].w*0.5
            #    length_front = obj[i].l*0.5
            #    length_rear = obj[i].l*0.5
            #    orientation = obj[i].o
            #elif mode == "srr520":
            #    if DistX > 0:
            #        #DistX = obj[i].x + obj[i].l*np.cos(orientation)*0.5
            #        #DistY = obj[i].y - obj[i].l*np.sin(orientation)*0.5
            #        width_left = obj[i].w*0.5
            #        width_right = obj[i].w*0.5
            #        length_front = obj[i].l*0.5
            #        length_rear = obj[i].l*0.5
            #        orientation = obj[i].o
            #    else:
            #        #DistX = obj[i].x - obj[i].l*np.cos(orientation)*0.5
            #        #DistY = obj[i].y + obj[i].l*np.sin(orientation)*0.5
            #        width_left = obj[i].w*0.5
            #        width_right = obj[i].w*0.5
            #        length_front = obj[i].l*0.5
            #        length_rear = obj[i].l*0.5
            #        orientation = obj[i].o

            # Four corner's points
            p0_x,p0_y,p1_x,p1_y,p2_x,p2_y,p3_x,p3_y = self.Rot_rectangle_SRR520(DistX, DistY, width_left, width_right, length_front, length_rear, orientation)

            # to picture coordinate
            pic_x0,pic_y0 = self.TransCoordinateNormal2pic(p0_x, p0_y)
            pic_x1,pic_y1 = self.TransCoordinateNormal2pic(p1_x, p1_y)
            pic_x2,pic_y2 = self.TransCoordinateNormal2pic(p2_x, p2_y)
            pic_x3,pic_y3 = self.TransCoordinateNormal2pic(p3_x, p3_y)

            if color == (0,0,255):
                Img = cv2.line(Img, (pic_x0,pic_y0), (pic_x1,pic_y1), color,thickness=2)
                Img = cv2.line(Img, (pic_x1,pic_y1), (pic_x2,pic_y2), color,thickness=2)
                Img = cv2.line(Img, (pic_x2,pic_y2), (pic_x3,pic_y3), color,thickness=2)
                Img = cv2.line(Img, (pic_x3,pic_y3), (pic_x0,pic_y0), color,thickness=1)
            else:
                Img = cv2.line(Img, (pic_x0,pic_y0), (pic_x1,pic_y1), color,thickness=1)
                Img = cv2.line(Img, (pic_x1,pic_y1), (pic_x2,pic_y2), color,thickness=1)
                Img = cv2.line(Img, (pic_x2,pic_y2), (pic_x3,pic_y3), color,thickness=1)
                Img = cv2.line(Img, (pic_x3,pic_y3), (pic_x0,pic_y0), color,thickness=2)

            #最悪　とってしまってもと思ったが、これを書いておくと検出点の直接的な比較ができるはず
            #おまじないとして残す
            pic_DistX,pic_DistY = self.TransCoordinateNormal2pic(DistX, DistY)
            Img = cv2.circle(Img, (int(pic_DistX),int(pic_DistY)), 3, color, -1)

        return Img




    ##=====================================
    ## Only view sensor's output
    ##=====================================
    def PlotFigure_ARS510_Obj_onlyView(self, ARS510_obj, Base = np.zeros(2)):
        Img = Base

        LAT_OFFSET2 = LAT_OFFSET+750
        LON_OFFSET2 = LON_OFFSET-450
        TF = -self.constYaw
        
        ## Objects
        #print(ARS510_obj[0].ui_num_of_used_object)
        for i in range(len(ARS510_obj)):
            category = ARS510_obj[i].e_classification
            #if category == 1:
            #if category == 0 or category == 1 or category == 2 or category == 3 or category == 4 or category == 5 or category == 6 or category == 7:
            DistX = ARS510_obj[i].f_dist_x
            DistY = ARS510_obj[i].f_dist_y
            width = ARS510_obj[i].f_width
            length = ARS510_obj[i].f_length
            orientation = ARS510_obj[i].f_orientation
            #print(DistX)

            # Four corner's points
            p0_x,p0_y,p1_x,p1_y,p2_x,p2_y,p3_x,p3_y = self.Rot_rectangle_ARS510(DistX, DistY, width, length, orientation)

            # to picture coordinate
            pic_x0,pic_y0 = self.TransCoordinateNormal2pic(p0_x, p0_y)
            pic_x1,pic_y1 = self.TransCoordinateNormal2pic(p1_x, p1_y)
            pic_x2,pic_y2 = self.TransCoordinateNormal2pic(p2_x, p2_y)
            pic_x3,pic_y3 = self.TransCoordinateNormal2pic(p3_x, p3_y)

            color = (15,230,240)# Yellow
            Img = cv2.line(Img, (pic_x0,pic_y0), (pic_x1,pic_y1), color,thickness=1)
            Img = cv2.line(Img, (pic_x1,pic_y1), (pic_x2,pic_y2), color,thickness=2)
            Img = cv2.line(Img, (pic_x2,pic_y2), (pic_x3,pic_y3), color,thickness=1)
            Img = cv2.line(Img, (pic_x3,pic_y3), (pic_x0,pic_y0), color,thickness=1)
            #Img = cv2.putText(Img,str(ARS510_obj[i].u_id), (pic_x3,pic_y3), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 1, cv2.LINE_AA)
            Img = cv2.putText(Img,str(round(ARS510_obj[i].f_probability_of_existence,2)), (pic_x3,pic_y3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
            Img = cv2.putText(Img,str(ARS510_obj[i].ui_life_cycles), (pic_x3,pic_y3+14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
            Img = cv2.putText(Img,str(category), (pic_x3,pic_y3+28), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
            # Img = cv2.putText(Img,str(round(np.rad2deg(orientation),2)), (pic_x3,pic_y3+14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
            # Img = cv2.putText(Img,str(round(np.sqrt(ARS510_obj[i].a_vrel_x**2+ARS510_obj[i].a_vrel_y**2),2)), (pic_x3,pic_y3+42), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

            ## Calc. rear x,y
            x = (p0_x + p3_x)*0.5
            y = (p0_y + p3_y)*0.5
            pic_DistX,pic_DistY = self.TransCoordinateNormal2pic(x, y)
            Img = cv2.circle(Img, (int(pic_DistX),int(pic_DistY)), 3, color, -1)

        return Img


    def PlotFigure_SRR520_Obj_onlyView(self, SRR520_obj, witchRadar, Base = np.zeros(2)):
        Img = Base

        LAT_OFFSET2 = LAT_OFFSET+750
        LON_OFFSET2 = LON_OFFSET-450
        TF = -self.constYaw
        
        ## Objects
        for i in range(len(SRR520_obj)):
            category = SRR520_obj[i].e_classification
            #if category == 1:
            #if category == 0 or category == 1 or category == 2 or category == 3 or category == 4 or category == 5 or category == 6 or category == 7:
            DistX = SRR520_obj[i].f_dist_x
            DistY = SRR520_obj[i].f_dist_y
            width_left = SRR520_obj[i].f_width_left
            width_right = SRR520_obj[i].f_width_right
            length_front = SRR520_obj[i].f_length_front
            length_rear = SRR520_obj[i].f_length_rear
            orientation = SRR520_obj[i].f_orientation

            # Four corner's points
            p0_x,p0_y,p1_x,p1_y,p2_x,p2_y,p3_x,p3_y = self.Rot_rectangle_SRR520(DistX, DistY, width_left, width_right, length_front, length_rear, orientation)

            # to picture coordinate
            pic_x0,pic_y0 = self.TransCoordinateNormal2pic(p0_x, p0_y)
            pic_x1,pic_y1 = self.TransCoordinateNormal2pic(p1_x, p1_y)
            pic_x2,pic_y2 = self.TransCoordinateNormal2pic(p2_x, p2_y)
            pic_x3,pic_y3 = self.TransCoordinateNormal2pic(p3_x, p3_y)

            if witchRadar == 'FL': color = (10,255,40)# Green
            if witchRadar == 'FR': color = (255,230,90)# Blue
            if witchRadar == 'RL': color = (15,120,235)# Orange
            if witchRadar == 'RR': color = (200,85,240)# Purple
            Img = cv2.line(Img, (pic_x0,pic_y0), (pic_x1,pic_y1), color,thickness=1)
            Img = cv2.line(Img, (pic_x1,pic_y1), (pic_x2,pic_y2), color,thickness=1)
            Img = cv2.line(Img, (pic_x2,pic_y2), (pic_x3,pic_y3), color,thickness=1)
            Img = cv2.line(Img, (pic_x3,pic_y3), (pic_x0,pic_y0), color,thickness=2)
            #Img = cv2.putText(Img,str(SRR520_obj[i].u_id), (pic_x3,pic_y3), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 1, cv2.LINE_AA)
            Img = cv2.putText(Img,str(round(SRR520_obj[i].f_probability_of_existence,2)), (pic_x3,pic_y3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
            Img = cv2.putText(Img,str(SRR520_obj[i].ui_life_cycles), (pic_x3,pic_y3+14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
            Img = cv2.putText(Img,str(category), (pic_x3,pic_y3+28), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
            #Img = cv2.putText(Img,str(round(np.rad2deg(orientation),2)), (pic_x3,pic_y3+14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

            ## Calc. rear x,y
            x = (p2_x + p1_x)*0.5
            y = (p2_y + p1_y)*0.5
            pic_DistX,pic_DistY = self.TransCoordinateNormal2pic(x, y)
            Img = cv2.circle(Img, (int(pic_DistX),int(pic_DistY)), 3, color, -1)

        return Img