import os
import shutil
import cv2
import argparse
import numpy as np
#from grid import grid

def grid(width, height, size, grid_text=False):
    grid_img = np.zeros((width, height,3), dtype=np.uint8)
    start=int((height-40)/size/2)
    scale_ = np.linspace(-start, start, int((height-40)/size+1), dtype=np.int8)
    scale = []

    for i in scale_:
        scale.append(str(i)+"m")
    
    #--draw ego car ---
    #grid_img[320-int(2.8*size),320-int(0.95*size):320+int(0.95*size),1] = 255
    #grid_img[320+int(1.7*size),320-int(0.95*size):320+int(0.95*size),1] = 255
    #grid_img[320-int(2.8*size):320+int(1.7*size), 320-int(0.95*size),1] = 255
    #grid_img[320-int(2.8*size):320+int(1.7*size), 320+int(0.95*size),1] = 255

    #grid_img[320-int(0.95*size):320+int(0.95*size),640-int(2.8*size),1] = 255
    #grid_img[320-int(0.95*size):320+int(0.95*size),640+int(1.7*size),1] = 255
    #grid_img[320-int(0.95*size),640-int(2.8*size):640+int(1.7*size), 1] = 255
    #grid_img[320+int(0.95*size),640-int(2.8*size):640+int(1.7*size), 1] = 255
    grid_img[320-int(0.95*size):320+int(0.95*size),780-int(2.8*size),1] = 255
    grid_img[320-int(0.95*size):320+int(0.95*size),780+int(1.7*size),1] = 255
    grid_img[320-int(0.95*size),780-int(2.8*size):780+int(1.7*size), 1] = 255
    grid_img[320+int(0.95*size),780-int(2.8*size):780+int(1.7*size), 1] = 255
    #
    #--draw grid line ---
    #for i in range(int(((height-40)/size+1)/5)-1):
    #    print("size check;", )
    #    grid_img[5*i*size+20,:,2] = 50

    #for i in range(int(((height-40)/size+1)/5)):
    #    grid_img[-5*i*size+620,:,2] = 50
    #    #grid_img[-5*i*size+320,:,2] = 50

    for i in range(int(((width-40)/size+1)/5)+3):
        #grid_img[:,  -5*i*size+1240,2] = 50
        grid_img[:,  -5*i*size+1530,2] = 50
        #grid_img[:,  -5*i*size+640,2] = 50
        grid_img[:,  -5*i*size+780,2] = 50

    if grid_text == True:
        for i in range(int((height-40)/size+1)):
            cv2.putText(grid_img, scale[i], (1260 - i*size+10,640), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,127), 1, cv2.LINE_AA)
            cv2.putText(grid_img, scale[i], (330, -i*size+630), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,127), 1, cv2.LINE_AA)

    return grid_img


class Plot:

    def __init__(self, grid=True):

        self.B_m = 0
        self.G_m = 255
        self.R_m = 0

        self.B_s =255
        self.G_s =0
        self.R_s =255


        imgRidar = np.zeros((640,640,3),dtype=np.uint8)
        #---Radar1_image_size-------------------------------------------------------------------------------------------------
        if grid:
            imgRidar[620 - 0*30,:,2] = 127
            imgRidar[620 - 1*30,:,2] = 127
            imgRidar[620 - 2*30,:,2] = 127
            imgRidar[620 - 3*30,:,2] = 127
            imgRidar[620 - 4*30,:,2] = 127
            imgRidar[620 - 5*30,:,2] = 127
            imgRidar[620 - 6*30,:,2] = 127
            imgRidar[620 - 7*30,:,2] = 127
            imgRidar[620 - 8*30,:,2] = 127
            imgRidar[620 - 9*30,:,2] = 127
            imgRidar[620 - 10*30,:,2] = 127
            imgRidar[620 - 11*30,:,2] = 127
            imgRidar[620 - 12*30,:,2] = 127
            imgRidar[620 - 13*30,:,2] = 127
            imgRidar[620 - 14*30,:,2] = 127
            imgRidar[620 - 15*30,:,2] = 127
            imgRidar[620 - 16*30,:,2] = 127
            imgRidar[620 - 17*30,:,2] = 127
            imgRidar[620 - 18*30,:,2] = 127
            imgRidar[620 - 19*30,:,2] = 127
            imgRidar[620 - 20*30,:,2] = 127

            imgRidar[620 - 9*30,-1*20+300+20:1*20+300+20,1] = 255
            imgRidar[620 - 11*30,-1*20+300+20:1*20+300+20,1] = 255
            imgRidar[620 - 11*30:620 - 9*30, 1*20+300+20,1] = 255
            imgRidar[620 - 11*30:620 - 9*30, -1*20+300+20,1] = 255
            #imgRidar[:, -1*20+300+20,1] = 127

            #---Radar_image_lattice_size(wedth1)----------------------------------------------------------------------------------
            #---Radar_image_lattice_size(wedth2)----------------------------------------------------------------------------------
            imgRidar[:,  0*30+300+20,2] = 127
            imgRidar[:, 1*30+300+20,2] = 127
            imgRidar[:, 2*30+300+20,2] = 127
            imgRidar[:, 3*30+300+20,2] = 127
            imgRidar[:, 4*30+300+20,2] = 127
            imgRidar[:, 5*30+300+20,2] = 127
            imgRidar[:, 6*30+300+20,2] = 127
            imgRidar[:, 7*30+300+20,2] = 127
            imgRidar[:, 8*30+300+20,2] = 127
            imgRidar[:, 9*30+300+20,2] = 127
            imgRidar[:, 10*30+300+20,2] = 127

            imgRidar[:,  0*30+300+20,2] = 127
            imgRidar[:, -1*30+300+20,2] = 127
            imgRidar[:, -2*30+300+20,2] = 127
            imgRidar[:, -3*30+300+20,2] = 127
            imgRidar[:, -4*30+300+20,2] = 127
            imgRidar[:, -5*30+300+20,2] = 127
            imgRidar[:, -6*30+300+20,2] = 127
            imgRidar[:, -7*30+300+20,2] = 127
            imgRidar[:, -8*30+300+20,2] = 127
            imgRidar[:, -9*30+300+20,2] = 127
            imgRidar[:, -10*30+300+20,2] = 127
  
        #---Radar_image_lattice_size(wedth2)----------------------------------------------------------------------------------
        #---Radar_image_m-----------------------------------------------------------------------------------------------------
        cv2.putText(imgRidar, ' 20m', ( 330,  10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' 18m', ( 330,  40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' 16m', ( 330, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' 14m', ( 330, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' 12m', ( 330, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' 10m', ( 330, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' 8m', ( 330,  190), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' 6m', ( 330, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' 4m', ( 330, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' 2m', ( 330, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, '  0m', ( 330, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)

        cv2.putText(imgRidar, ' -20m', ( 330,  610), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' -18m', ( 330,  580), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' -16m', ( 330, 550), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' -14m', ( 330, 520), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' -12m', ( 330, 490), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' -10m', ( 330, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' -8m', ( 330,  430), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' -6m', ( 330, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' -4m', ( 330, 370), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar, ' -2m', ( 330, 340), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        
        imgRidar2 = np.zeros((640,640,3),dtype=np.uint8)

        imgRidar2[620 - 6*60, -1*30+300+20:1*30+300+20,1] = 255
        imgRidar2[620 - 4*60,-1*30+300+20:1*30+300+20,1] = 255
        imgRidar2[620 - 6*60:620 - 4*60, 1*30+300+20,1] = 255
        imgRidar2[620 - 6*60:620 - 4*60, -1*30+300+20,1] = 255

        imgRidar2[620 - 0*60,:,2] = 127
        imgRidar2[620 - 1*60,:,2] = 127
        imgRidar2[620 - 2*60,:,2] = 127
        imgRidar2[620 - 3*60,:,2] = 127
        imgRidar2[620 - 4*60,:,2] = 127
        imgRidar2[620 - 5*60,:,2] = 127
        imgRidar2[620 - 6*60,:,2] = 127
        imgRidar2[620 - 7*60,:,2] = 127
        imgRidar2[620 - 8*60,:,2] = 127
        imgRidar2[620 - 9*60,:,2] = 127
        imgRidar2[620 - 10*60,:,2] = 127
        
        imgRidar2[:,  0*60+300+20,2] = 127
        imgRidar2[:, 1*60+300+20,2] = 127
        imgRidar2[:, 2*60+300+20,2] = 127
        imgRidar2[:, 3*60+300+20,2] = 127
        imgRidar2[:, 4*60+300+20,2] = 127
        imgRidar2[:, 5*60+300+20,2] = 127

        imgRidar2[:,  0*60+300+20,2] = 127
        imgRidar2[:, -1*60+300+20,2] = 127
        imgRidar2[:, -2*60+300+20,2] = 127
        imgRidar2[:, -3*60+300+20,2] = 127
        imgRidar2[:, -4*60+300+20,2] = 127
        imgRidar2[:, -5*60+300+20,2] = 127

        cv2.putText(imgRidar2, ' 10m', ( 330, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar2, ' 8m', ( 330,  70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar2, ' 6m', ( 330, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar2, ' 4m', ( 330, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar2, ' 2m', ( 330, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar2, ' 0m', ( 330, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)

        cv2.putText(imgRidar2, ' -10m', ( 330, 600), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar2, ' -8m', ( 330,  540), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar2, ' -6m', ( 330, 490), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar2, ' -4m', ( 330, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar2, ' -2m', ( 330, 370), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        
        cv2.putText(imgRidar2, ' 10m', (600, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar2, ' 8m', ( 540, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar2, ' 6m', ( 490, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar2, ' 4m', ( 430, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar2, ' 2m', ( 370, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        #cv2.putText(imgRidar2, '  0m', ( 330, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)

        cv2.putText(imgRidar2, ' -10m', ( 10, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar2, ' -8m', ( 70, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar2, ' -6m', ( 130, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar2, ' -4m', ( 190, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar2, ' -2m', ( 250, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)

        

        
        #---Radar_image_lattice_size(height)----------------------------------------------------------------------------------
        imgRidar3 = np.zeros((640+400,640,3),dtype=np.uint8)
        imgRidar3[520 - 0*50,:,2] = 127
        imgRidar3[520 - 1*50,:,2] = 127
        imgRidar3[520 - 2*50,:,2] = 127
        imgRidar3[520 - 3*50,:,2] = 127
        imgRidar3[520 - 4*50,:,2] = 127
        imgRidar3[520 - 5*50,:,2] = 127
        imgRidar3[520 - 6*50,:,2] = 127
        imgRidar3[520 - 7*50,:,2] = 127
        imgRidar3[520 - 8*50,:,2] = 127
        imgRidar3[520 - 9*50,:,2] = 127
        imgRidar3[520 - 10*50,:,2] = 127
        #imgRidar3[520 - 0*50,:,2] = 127
        imgRidar3[520 + 1*50,:,2] = 127
        imgRidar3[520 + 2*50,:,2] = 127
        imgRidar3[520 + 3*50,:,2] = 127
        imgRidar3[520 + 4*50,:,2] = 127
        imgRidar3[520 + 5*50,:,2] = 127
        imgRidar3[520 + 6*50,:,2] = 127
        imgRidar3[520 + 7*50,:,2] = 127
        imgRidar3[520 + 8*50,:,2] = 127
        imgRidar3[520 + 9*50,:,2] = 127
        imgRidar3[520 + 10*50,:,2] = 127
        #imgRidar3[:, -1*20+300+20,1] = 127

        #---Radar_image_lattice_size(wedth1)----------------------------------------------------------------------------------
        #---Radar_image_lattice_size(wedth2)----------------------------------------------------------------------------------
        imgRidar3[:,  0*50+300+20,2] = 127
        imgRidar3[:, 1*50+300+20,2] = 127
        imgRidar3[:, 2*50+300+20,2] = 127
        imgRidar3[:, 3*50+300+20,2] = 127
        imgRidar3[:, 4*50+300+20,2] = 127
        imgRidar3[:, 5*50+300+20,2] = 127

        imgRidar3[:,  0*50+300+20,2] = 127
        imgRidar3[:, -1*50+300+20,2] = 127
        imgRidar3[:, -2*50+300+20,2] = 127
        imgRidar3[:, -3*50+300+20,2] = 127
        imgRidar3[:, -4*50+300+20,2] = 127
        imgRidar3[:, -5*50+300+20,2] = 127
  
        #---Radar_image_lattice_size(wedth2)----------------------------------------------------------------------------------
        #---Radar_image_m-----------------------------------------------------------------------------------------------------
        cv2.putText(imgRidar3, ' 100m', ( 330,510-100*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' 90m', ( 330, 510-90*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' 80m', ( 330, 510-80*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' 70m', ( 330, 510-70*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' 60m', ( 330, 510-60*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' 50m', ( 330, 510-50*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' 40m', ( 330, 510-40*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' 30m', ( 330, 510-30*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' 20m', ( 330, 510-20*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' 10m', ( 330, 510-10*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, '  0m', ( 330, 510-0*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)

        cv2.putText(imgRidar3, ' -100m', ( 330,  510+100*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' -90m', ( 330,  510+90*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' -80m', ( 330, 510+80*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' -70m', ( 330, 510+70*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' -60m', ( 330, 510+60*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' -50m', ( 330, 510+50*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' -40m', ( 330, 510+40*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' -30m', ( 330, 510+30*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' -20m', ( 330, 510+20*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
        cv2.putText(imgRidar3, ' -10m', ( 330, 510+10*5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)

        imgRidar = cv2.circle(imgRidar,(320, 320), 3 ,(255,0,0 ),-1)
        imgRidar2 = cv2.circle(imgRidar2,(320, 320), 3 ,(255,0,0 ),-1)
        imgRidar3 = cv2.circle(imgRidar3,(320, 320), 3 ,(255,0,0 ),-1)
        
        self.imgRidar  = imgRidar
        self.imgRidar2  = imgRidar2
        self.imgRidar3  = imgRidar3
    
        
    def plot_cluster(self,sonar,sonar2,sonar3,lidar,posf,posl,posr,posr_l,posr_r,state_f,state_l,state_r,state_r_l,state_r_r,speed, vis_scale):
        
        imgRidar = self.imgRidar.copy()
        imgRidar2 = self.imgRidar2.copy()
        imgRidar3 = self.imgRidar3.copy()
        imgRidar_size = grid(640, 1560, vis_scale)
        #imgRidar_size = grid(640, 1280, vis_scale)

        speed=str(speed)

        cv2.putText(imgRidar, ("speed="+speed), ( 500,  30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)

        if len(sonar)>0 and len(sonar2)>0 and len(sonar3)>0:

            x=sonar[:,0]
            y=sonar[:,1]
            D=sonar[:,5]

            vx=sonar[:,2]
            vy=sonar[:,3]
            
            ZT=sonar2[:,4]
            Zb=sonar2[:,5]

            Ldx=sonar2[:,0]
            Ldy=-sonar2[:,1]
            Rdx=sonar2[:,2]
            Rdy=-sonar2[:,3]

            
            ID=sonar3[:,4]
            ob=sonar3[:,3]

                    
            for i in range(sonar.shape[0]):

                if ob[i]==0:
        
                    imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (255, 0, 255), markerType=cv2.MARKER_TILTED_CROSS, markerSize=15)
                    imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (255, 0, 255), markerType=cv2.MARKER_TILTED_CROSS, markerSize=15)

                    cv2.putText(imgRidar, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))), (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                    cv2.putText(imgRidar2, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))), (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)

                    cv2.putText(imgRidar, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))+"/"+str(vx[i])+"/"+str(vy[i])), (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                    cv2.putText(imgRidar2, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))+"/"+str(vx[i])+"/"+str(vy[i])), (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                   
                elif ob[i]==1:

                    imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (255, 0, 0), markerType=cv2.MARKER_STAR, markerSize=15)
                    imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (255, 0, 0), markerType=cv2.MARKER_STAR, markerSize=15)
                    cv2.putText(imgRidar, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))), (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                    cv2.putText(imgRidar2, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))), (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)

                elif ob[i]==2:

                    imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (0, 255, 0), markerType=cv2.MARKER_STAR, markerSize=15)
                    imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (0, 255, 0), markerType=cv2.MARKER_STAR, markerSize=15)
                    cv2.putText(imgRidar, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))), (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                    cv2.putText(imgRidar2, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))), (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                
                elif ob[i]==3:

                    imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (255, 0, 255), markerType=cv2.MARKER_STAR, markerSize=15)
                    imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (255, 0, 255), markerType=cv2.MARKER_STAR, markerSize=15)
                    cv2.putText(imgRidar, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))), (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                    cv2.putText(imgRidar2, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))), (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                elif ob[i]==4:

                    imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (0, 255, 255), markerType=cv2.MARKER_STAR, markerSize=15)
                    imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (0, 255, 255), markerType=cv2.MARKER_STAR, markerSize=15)
                    cv2.putText(imgRidar, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))), (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                    cv2.putText(imgRidar2, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))), (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                elif ob[i]==5:

                    imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (255, 0, 0),  markerType=cv2.MARKER_DIAMOND, markerSize=15,thickness = 1)
                    imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (255, 0, 0),  markerType=cv2.MARKER_DIAMOND, markerSize=15,thickness = 1)
                    cv2.putText(imgRidar, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))), (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                    cv2.putText(imgRidar2, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))), (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)

                elif ob[i]==6:

                    imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (0, 255, 0),  markerType=cv2.MARKER_DIAMOND, markerSize=15,thickness = 1)
                    imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (0, 255, 0),  markerType=cv2.MARKER_DIAMOND, markerSize=15,thickness = 1)
                    cv2.putText(imgRidar, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))), (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                    cv2.putText(imgRidar2, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))), (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                
                elif ob[i]==7:

                    imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (255, 0, 255),  markerType=cv2.MARKER_DIAMOND, markerSize=15,thickness = 1)
                    imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (255, 0, 255),  markerType=cv2.MARKER_DIAMOND, markerSize=15,thickness = 1)
                    cv2.putText(imgRidar, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))), (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                    cv2.putText(imgRidar2, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))), (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                
                elif ob[i]==8:

                    cv2.line(imgRidar, ( int((y[i]+Ldy[i])*15+300+20), int(-(x[i]+Ldx[i])*15+300+20) ), ( int((y[i]+Ldy[i])*15+300+20), int(-(x[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar, ( int((y[i]+Ldy[i])*15+300+20), int(-(x[i])*15+300+20) ), ( int((y[i])*15+300+20), int(-(x[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar, ( int((y[i])*15+300+20), int(-(x[i])*15+300+20) ), ( int((y[i]+Rdy[i])*15+300+20), int(-(x[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar, ( int((y[i]+Rdy[i])*15+300+20), int(-(x[i])*15+300+20) ), ( int((y[i]+Rdy[i])*15+300+20), int(-(x[i]+Rdx[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)

                    cv2.line(imgRidar2, ( int((y[i]+Ldy[i])*30+300+20), int(-(x[i]+Ldx[i])*30+300+20) ), ( int((y[i]+Ldy[i])*30+300+20), int(-(x[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar2, ( int((y[i]+Ldy[i])*30+300+20), int(-(x[i])*30+300+20) ), ( int((y[i])*30+300+20), int(-(x[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar2, ( int((y[i])*30+300+20), int(-(x[i])*30+300+20) ), ( int((y[i]+Rdy[i])*30+300+20), int(-(x[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar2, ( int((y[i]+Rdy[i])*30+300+20), int(-(x[i])*30+300+20) ), ( int((y[i]+Rdy[i])*30+300+20), int(-(x[i]+Rdx[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)


                    imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (0, 255, 255),  markerType=cv2.MARKER_DIAMOND, markerSize=15,thickness = 1)
                    imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (0, 255, 255),  markerType=cv2.MARKER_DIAMOND, markerSize=15,thickness = 1)
                    
                    cv2.putText(imgRidar, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))+"/"+str(vx[i])+"/"+str(vy[i])), (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                    cv2.putText(imgRidar2, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))+"/"+str(vx[i])+"/"+str(vy[i])), (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                
                elif ob[i]==9:

                    cv2.line(imgRidar, ( int((y[i]+Ldy[i])*15+300+20), int(-(x[i]+Ldx[i])*15+300+20) ), ( int((y[i]+Ldy[i])*15+300+20), int(-(x[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar, ( int((y[i]+Ldy[i])*15+300+20), int(-(x[i])*15+300+20) ), ( int((y[i])*15+300+20), int(-(x[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar, ( int((y[i])*15+300+20), int(-(x[i])*15+300+20) ), ( int((y[i]+Rdy[i])*15+300+20), int(-(x[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar, ( int((y[i]+Rdy[i])*15+300+20), int(-(x[i])*15+300+20) ), ( int((y[i]+Rdy[i])*15+300+20), int(-(x[i]+Rdx[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)

                    cv2.line(imgRidar2, ( int((y[i]+Ldy[i])*30+300+20), int(-(x[i]+Ldx[i])*30+300+20) ), ( int((y[i]+Ldy[i])*30+300+20), int(-(x[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar2, ( int((y[i]+Ldy[i])*30+300+20), int(-(x[i])*30+300+20) ), ( int((y[i])*30+300+20), int(-(x[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar2, ( int((y[i])*30+300+20), int(-(x[i])*30+300+20) ), ( int((y[i]+Rdy[i])*30+300+20), int(-(x[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar2, ( int((y[i]+Rdy[i])*30+300+20), int(-(x[i])*30+300+20) ), ( int((y[i]+Rdy[i])*30+300+20), int(-(x[i]+Rdx[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)

                    imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (255, 0, 0),  markerType=cv2.MARKER_TRIANGLE_UP, markerSize=15,thickness = 1)
                    imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (255, 0, 0),  markerType=cv2.MARKER_TRIANGLE_UP, markerSize=15,thickness = 1)
                    

                    cv2.putText(imgRidar, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))+"/"+str(vx[i])+"/"+str(vy[i])), (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                    cv2.putText(imgRidar2, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))+"/"+str(vx[i])+"/"+str(vy[i])), (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                
                elif ob[i]==10:

                    cv2.line(imgRidar, ( int((y[i]+Ldy[i])*15+300+20), int(-(x[i]+Ldx[i])*15+300+20) ), ( int((y[i]+Ldy[i])*15+300+20), int(-(x[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar, ( int((y[i]+Ldy[i])*15+300+20), int(-(x[i])*15+300+20) ), ( int((y[i])*15+300+20), int(-(x[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar, ( int((y[i])*15+300+20), int(-(x[i])*15+300+20) ), ( int((y[i]+Rdy[i])*15+300+20), int(-(x[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar, ( int((y[i]+Rdy[i])*15+300+20), int(-(x[i])*15+300+20) ), ( int((y[i]+Rdy[i])*15+300+20), int(-(x[i]+Rdx[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)

                    cv2.line(imgRidar2, ( int((y[i]+Ldy[i])*30+300+20), int(-(x[i]+Ldx[i])*30+300+20) ), ( int((y[i]+Ldy[i])*30+300+20), int(-(x[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar2, ( int((y[i]+Ldy[i])*30+300+20), int(-(x[i])*30+300+20) ), ( int((y[i])*30+300+20), int(-(x[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar2, ( int((y[i])*30+300+20), int(-(x[i])*30+300+20) ), ( int((y[i]+Rdy[i])*30+300+20), int(-(x[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar2, ( int((y[i]+Rdy[i])*30+300+20), int(-(x[i])*30+300+20) ), ( int((y[i]+Rdy[i])*30+300+20), int(-(x[i]+Rdx[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)

                    imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (0, 255, 0),  markerType=cv2.MARKER_TRIANGLE_UP, markerSize=15,thickness = 1)
                    imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (0, 255, 0),  markerType=cv2.MARKER_TRIANGLE_UP, markerSize=15,thickness = 1)
                    

                    cv2.putText(imgRidar, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))+"/"+str(vx[i])+"/"+str(vy[i])), (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                    cv2.putText(imgRidar2, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))+"/"+str(vx[i])+"/"+str(vy[i])), (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                
                elif ob[i]==11:

                    cv2.line(imgRidar, ( int((y[i]+Ldy[i])*15+300+20), int(-(x[i]+Ldx[i])*15+300+20) ), ( int((y[i]+Ldy[i])*15+300+20), int(-(x[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar, ( int((y[i]+Ldy[i])*15+300+20), int(-(x[i])*15+300+20) ), ( int((y[i])*15+300+20), int(-(x[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar, ( int((y[i])*15+300+20), int(-(x[i])*15+300+20) ), ( int((y[i]+Rdy[i])*15+300+20), int(-(x[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar, ( int((y[i]+Rdy[i])*15+300+20), int(-(x[i])*15+300+20) ), ( int((y[i]+Rdy[i])*15+300+20), int(-(x[i]+Rdx[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)

                    cv2.line(imgRidar2, ( int((y[i]+Ldy[i])*30+300+20), int(-(x[i]+Ldx[i])*30+300+20) ), ( int((y[i]+Ldy[i])*30+300+20), int(-(x[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar2, ( int((y[i]+Ldy[i])*30+300+20), int(-(x[i])*30+300+20) ), ( int((y[i])*30+300+20), int(-(x[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar2, ( int((y[i])*30+300+20), int(-(x[i])*30+300+20) ), ( int((y[i]+Rdy[i])*30+300+20), int(-(x[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar2, ( int((y[i]+Rdy[i])*30+300+20), int(-(x[i])*30+300+20) ), ( int((y[i]+Rdy[i])*30+300+20), int(-(x[i]+Rdx[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)

                    imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (255, 0, 255),  markerType=cv2.MARKER_TRIANGLE_UP, markerSize=15,thickness = 1)
                    imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (255, 0, 255),  markerType=cv2.MARKER_TRIANGLE_UP, markerSize=15,thickness = 1)
                    

                    cv2.putText(imgRidar, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))+"/"+str(vx[i])+"/"+str(vy[i])), (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                    cv2.putText(imgRidar2, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))+"/"+str(vx[i])+"/"+str(vy[i])), (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                
                elif ob[i]==12:

                    cv2.line(imgRidar, ( int((y[i]+Ldy[i])*15+300+20), int(-(x[i]+Ldx[i])*15+300+20) ), ( int((y[i]+Ldy[i])*15+300+20), int(-(x[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar, ( int((y[i]+Ldy[i])*15+300+20), int(-(x[i])*15+300+20) ), ( int((y[i])*15+300+20), int(-(x[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar, ( int((y[i])*15+300+20), int(-(x[i])*15+300+20) ), ( int((y[i]+Rdy[i])*15+300+20), int(-(x[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar, ( int((y[i]+Rdy[i])*15+300+20), int(-(x[i])*15+300+20) ), ( int((y[i]+Rdy[i])*15+300+20), int(-(x[i]+Rdx[i])*15+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)

                    cv2.line(imgRidar2, ( int((y[i]+Ldy[i])*30+300+20), int(-(x[i]+Ldx[i])*30+300+20) ), ( int((y[i]+Ldy[i])*30+300+20), int(-(x[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar2, ( int((y[i]+Ldy[i])*30+300+20), int(-(x[i])*30+300+20) ), ( int((y[i])*30+300+20), int(-(x[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar2, ( int((y[i])*30+300+20), int(-(x[i])*30+300+20) ), ( int((y[i]+Rdy[i])*30+300+20), int(-(x[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
                    cv2.line(imgRidar2, ( int((y[i]+Rdy[i])*30+300+20), int(-(x[i])*30+300+20) ), ( int((y[i]+Rdy[i])*30+300+20), int(-(x[i]+Rdx[i])*30+300+20) ), (0, 255, 0), thickness=2, lineType=cv2.LINE_4)

                    imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (0, 255, 255),  markerType=cv2.MARKER_TRIANGLE_UP, markerSize=15,thickness = 1)
                    imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (0, 255, 255),  markerType=cv2.MARKER_TRIANGLE_UP, markerSize=15,thickness = 1)
                    
                    cv2.putText(imgRidar, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))+"/"+str(vx[i])+"/"+str(vy[i])), (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                    cv2.putText(imgRidar2, (" "+str(D[i])+"/"+str(np.round((ZT[i]-Zb[i]),2))+"/"+str(vx[i])+"/"+str(vy[i])), (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                else:
                    imgRidar = cv2.circle(imgRidar,(int(y[i]*15)+300+20, int(-x[i]*15)+300+20), 3 ,(0,255,0 ),-1)
                    imgRidar2 = cv2.circle(imgRidar2,(int(y[i]*30)+300+20, int(-x[i]*30)+300+20), 3 ,(0,255,0 ),-1)
                
        
        if len(lidar)>0: 
    
            x=lidar[:,0]
            y=-lidar[:,1]
            z=lidar[:,2]

            for i in range(lidar.shape[0]):
                if z[i] <= -1.5:
                    imgRidar = cv2.circle(imgRidar,(int(y[i]*15)+300+20, int(-x[i]*15)+300+20), 1 ,(70,70,70),1)
                    imgRidar2 = cv2.circle(imgRidar2,(int(y[i]*30)+300+20, int(-x[i]*30)+300+20), 1 ,(70,70,70),1)
                    imgRidar3 = cv2.circle(imgRidar3,(int(y[i]*5)+300+20, int(-x[i]*5)+520), 1 ,(70,70,70,),1)
                    #imgRidar_size = cv2.circle(imgRidar_size,(int(y[i]*size)+640, int(-x[i]*5)+320), 1 ,(30,30,30),-1)
                    imgRidar_size = cv2.circle(imgRidar_size,(int(x[i]*vis_scale)+780, int(y[i]*vis_scale)+320), 1 ,(70,70,70),1)
                if -1.5<z[i]<-1.0:
                    imgRidar = cv2.circle(imgRidar,(int(y[i]*15)+300+20, int(-x[i]*15)+300+20), 1 ,(0,255,255 ),1)
                    imgRidar2 = cv2.circle(imgRidar2,(int(y[i]*30)+300+20, int(-x[i]*30)+300+20), 1 ,(0,255,255 ),1)
                    imgRidar3 = cv2.circle(imgRidar3,(int(y[i]*5)+(300+20)*1, int(-x[i]*5)+520), 1 ,(0,255,255 ),1)
                    imgRidar_size = cv2.circle(imgRidar_size,(int(x[i]*vis_scale)+780, int(y[i]*vis_scale)+320), 1 ,(0,255,255 ),1)
                
                elif -1.0<z[i]:
                    imgRidar = cv2.circle(imgRidar,(int(y[i]*15)+300+20, int(-x[i]*15)+300+20), 1 ,(255,255,0 ),1)
                    imgRidar2 = cv2.circle(imgRidar2,(int(y[i]*30)+300+20, int(-x[i]*30)+300+20), 1 ,(255,255,0 ),1)
                    imgRidar3 = cv2.circle(imgRidar3,(int(y[i]*5)+(300+20)*1, int(-x[i]*5)+520), 1 ,(255,255,0 ),1)
                    imgRidar_size = cv2.circle(imgRidar_size,(int(x[i]*vis_scale)+780, int(y[i]*vis_scale)+320), 1 ,(255,255,0),1)
        
        if len(posf)>0:
            x=posf[:,2]
            y=posf[:,0]
    
            for i in range(posf.shape[0]):
                
                if (abs(x[i])<=20) and (abs(y[i])<=20): 
                    if state_f[i]==0:

                        imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (self.B_m, self.G_m, self.R_m), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)
                        imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (self.B_m, self.G_m, self.R_m), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)
                        #imgRidar3 =cv2.drawMarker(imgRidar3, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (0, 0, 255), markerType=cv2.MARKER_TILTED_CROSS, markerSize=5)

                        #imgRidar = cv2.circle(imgRidar,(int(y[i]*15)+300+20, int(-x[i]*15)+300+20), 2 ,(0,0,255 ),-1)
                        #imgRidar2 = cv2.circle(imgRidar2,(int(y[i]*30)+300+20, int(-x[i]*30)+300+20), 2 ,(0,0,255 ),-1)
                    elif state_f[i]==1:

                        imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (self.B_s, self.G_s, self.R_s), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)
                        imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (self.B_s, self.G_s, self.R_s), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)

                        #imgRidar = cv2.circle(imgRidar,(int(y[i]*15)+300+20, int(-x[i]*15)+300+20), 2 ,(0,255,0 ),-1)
                        #imgRidar2 = cv2.circle(imgRidar2,(int(y[i]*30)+300+20, int(-x[i]*30)+300+20), 2 ,(0,255,0 ),-1)

                
                if (abs(x[i])<=200) and (abs(y[i])<=60): 
                    if state_f[i]==0:
                        imgRidar3 =cv2.drawMarker(imgRidar3, (int(y[i]*5)+(300+20)*1, int(-x[i]*5)+520), (self.B_m, self.G_m, self.R_m), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)

                    elif state_f[i]==1:

                        imgRidar3 =cv2.drawMarker(imgRidar3, (int(y[i]*5)+(300+20)*1, int(-x[i]*5)+520), (self.B_s, self.G_s, self.R_s), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)

        
        if len(posl)>0:
            x=posl[:,2]
            y=posl[:,0]
    
            for i in range(posl.shape[0]):

                if (abs(x[i])<=20) and (abs(y[i])<=20): 

                    if state_l[i]==0:
                        imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (self.B_m, self.G_m, self.R_m), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)
                        imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (self.B_m, self.G_m, self.R_m), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)

                        #imgRidar = cv2.circle(imgRidar,(int(y[i]*15)+300+20, int(-x[i]*15)+300+20), 2 ,(0,0,255 ),-1)
                        #imgRidar2 = cv2.circle(imgRidar2,(int(y[i]*30)+300+20, int(-x[i]*30)+300+20), 2 ,(0,0,255 ),-1)
                    elif state_l[i]==1:

                        imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (self.B_s, self.G_s, self.R_s), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)
                        imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (self.B_s, self.G_s, self.R_s), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)

                        #imgRidar = cv2.circle(imgRidar,(int(y[i]*15)+300+20, int(-x[i]*15)+300+20), 2 ,(0,255,0 ),-1)
                        #imgRidar2 = cv2.circle(imgRidar2,(int(y[i]*30)+300+20, int(-x[i]*30)+300+20), 2 ,(0,255,0 ),-1)

                if (abs(x[i])<=200) and (abs(y[i])<=60): 
                    if state_l[i]==0:
                        imgRidar3 =cv2.drawMarker(imgRidar3, (int(y[i]*5)+(300+20)*1, int(-x[i]*5)+520), (self.B_m, self.G_m, self.R_m), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)

                    elif state_l[i]==1:

                        imgRidar3 =cv2.drawMarker(imgRidar3, (int(y[i]*5)+(300+20)*1, int(-x[i]*5)+520), (self.B_s, self.G_s, self.R_s), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)
            
        if len(posr)>0:
            x=posr[:,2]
            y=posr[:,0]
    
            for i in range(posr.shape[0]):
                
                if (abs(x[i])<=20) and (abs(y[i])<=20): 

                    if state_r[i]==0:
                        imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (self.B_m, self.G_m, self.R_m), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)
                        imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (self.B_m, self.G_m, self.R_m), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)

                        #imgRidar = cv2.circle(imgRidar,(int(y[i]*15)+300+20, int(-x[i]*15)+300+20), 2 ,(0,0,255 ),-1)
                        #imgRidar2 = cv2.circle(imgRidar2,(int(y[i]*30)+300+20, int(-x[i]*30)+300+20), 2 ,(0,0,255 ),-1)
                    elif state_r[i]==1:
                        imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (self.B_s, self.G_s, self.R_s), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)
                        imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (self.B_s, self.G_s, self.R_s), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)


                        #imgRidar = cv2.circle(imgRidar,(int(y[i]*15)+300+20, int(-x[i]*15)+300+20), 2 ,(0,255,0 ),-1)
                        #imgRidar2 = cv2.circle(imgRidar2,(int(y[i]*30)+300+20, int(-x[i]*30)+300+20), 2 ,(0,255,0 ),-1)
                if (abs(x[i])<=200) and (abs(y[i])<=60): 
                    if state_r[i]==0:
                        imgRidar3 =cv2.drawMarker(imgRidar3, (int(y[i]*5)+300+20, int(-x[i]*5)+520), (self.B_m, self.G_m, self.R_m), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)

                    elif state_r[i]==1:

                        imgRidar3 =cv2.drawMarker(imgRidar3, (int(y[i]*5)+300+20, int(-x[i]*5)+520), (self.B_s, self.G_s, self.R_s), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)

        if len(posr_l)>0:
            x=posr_l[:,2]
            y=posr_l[:,0]
    
            for i in range(posr_l.shape[0]):

                if (abs(x[i])<=20) and (abs(y[i])<=20): 

                    if state_r_l[i]==0:
                        imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (self.B_m, self.G_m, self.R_m), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)
                        imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (self.B_m, self.G_m, self.R_m), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)

                        #imgRidar = cv2.circle(imgRidar,(int(y[i]*15)+300+20, int(-x[i]*15)+300+20), 2 ,(0,0,255 ),-1)
                        #imgRidar2 = cv2.circle(imgRidar2,(int(y[i]*30)+300+20, int(-x[i]*30)+300+20), 2 ,(0,0,255 ),-1)
                    elif state_r_l[i]==1:
                        imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (self.B_s, self.G_s, self.R_s), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)
                        imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (self.B_s, self.G_s, self.R_s), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)

                        #imgRidar = cv2.circle(imgRidar,(int(y[i]*15)+300+20, int(-x[i]*15)+300+20), 2 ,(0,255,0 ),-1)
                        #imgRidar2 = cv2.circle(imgRidar2,(int(y[i]*30)+300+20, int(-x[i]*30)+300+20), 2 ,(0,255,0 ),-1)
                
                if (abs(x[i])<=200) and (abs(y[i])<=60): 
                    if state_r_l[i]==0:
                        imgRidar3 =cv2.drawMarker(imgRidar3, (int(y[i]*5)+300+20, int(-x[i]*5)+520), (self.B_m, self.G_m, self.R_m), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)

                    elif state_r_l[i]==1:

                        imgRidar3 =cv2.drawMarker(imgRidar3, (int(y[i]*5)+300+20, int(-x[i]*5)+520), (self.B_s, self.G_s, self.R_s), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)
        
        if len(posr_r)>0:
            x=posr_r[:,2]
            y=posr_r[:,0]
    
            for i in range(posr_r.shape[0]):

                if (abs(x[i])<=20) and (abs(y[i])<=20): 

                    if state_r_r[i]==0:
                        imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (self.B_m, self.G_m, self.R_m), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)
                        imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (self.B_m, self.G_m, self.R_m), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)

                        #imgRidar = cv2.circle(imgRidar,(int(y[i]*15)+300+20, int(-x[i]*15)+300+20), 2 ,(0,0,255 ),-1)
                        #imgRidar2 = cv2.circle(imgRidar2,(int(y[i]*30)+300+20, int(-x[i]*30)+300+20), 2 ,(0,0,255 ),-1)
                    elif state_r_r[i]==1:
                        imgRidar =cv2.drawMarker(imgRidar, (int(y[i]*15)+300+20, int(-x[i]*15)+300+20), (self.B_s, self.G_s, self.R_s), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)
                        imgRidar2 =cv2.drawMarker(imgRidar2, (int(y[i]*30)+300+20, int(-x[i]*30)+300+20), (self.B_s, self.G_s, self.R_s), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)

                        
                        #imgRidar = cv2.circle(imgRidar,(int(y[i]*15)+300+20, int(-x[i]*15)+300+20), 2 ,(0,255,0 ),-1)
                        #imgRidar2 = cv2.circle(imgRidar2,(int(y[i]*30)+300+20, int(-x[i]*30)+300+20), 2 ,(0,255,0 ),-1)
                if (abs(x[i])<=200) and (abs(y[i])<=60): 
                    if state_r_r[i]==0:
                        imgRidar3 =cv2.drawMarker(imgRidar3, (int(y[i]*5)+300+20, int(-x[i]*5)+520), (self.B_m, self.G_m, self.R_m), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)

                    elif state_r_r[i]==1:

                        imgRidar3 =cv2.drawMarker(imgRidar3, (int(y[i]*5)+300+20, int(-x[i]*5)+520), (self.B_s, self.G_s, self.R_s), markerType=cv2.MARKER_TILTED_CROSS, markerSize=7)
                
           
        return imgRidar,imgRidar3,imgRidar2, imgRidar_size
