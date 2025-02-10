from tkinter import *
from PIL import Image, ImageOps, ImageTk
import os
import numpy as np
from matplotlib import pyplot as plt
from enum import Enum
import pathlib
import csv
import cv2
import platform
import random
from numpy import sin, cos, arctan2, pi, sqrt

from DataLoader import Loader
from config.option_parser import option_parser
#from PIL import Image, ImageTK

#--------add argparse for main_fusion function----------#
args = option_parser().get_option()
ano_dir = args.annot_base_dir
#-------------------------------------------------------#

colors = ['red', 'green', 'blue', 'cyan', 'yellow', 'magenta', 'white', 'pink'
, 'green yellow', 'orange', 'gray', 'sky blue', 'purple', 'saddle brown', 'steel blue']

sensor_color = {"SRR520_FL":"white",
         "SRR520_FR":"white",
         "SRR520_RR":"white",
         "SRR520_RL":"white",
         "ARS510":"magenta",
         "SVC220":"yellow"
         }

random.seed(0)
random.shuffle(colors)

if platform.system() == 'Windows':
    active_state = 'normal'
else:
    active_state = 'active'

#----------------- Target Sensor List ----------------#
sensor_list = {"SVC220_Front":None,
               "SVC220_Left":None,
               "SVC220_Right":None,
               "SVC220_Rear":None,
               "ARS520_Front??":None,
               "SONAR":None
}

vis_scale = 10
scale = 3
center_u = 780
#center_u = 640
center_v = 320
#-------------------------------------------------------#

#---------------- define save file name ----------------#
file_name = args.data_name + "/annotation_toshow.csv"
#-------------------------------------------------------#

#--------Modify Image Format For TK Module--------------#
def cv2totk(img):
    #bgr to rgb 
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    pil = Image.fromarray(rgb)
    tkimg = ImageTk.PhotoImage(pil)
    return tkimg
#-------------------------------------------------------#

#------------- Manuver Information Setup ----------------#
class DrawStatus(Enum):
    IDLE = 0
    DRAWING = 1
    TAIL = 2
    SIDE = 3
    FRONT = 4
    REAR = 5

class InputStatus(Enum):
    SELECT = 0
    MAKE = 1
    MOVE = 2
    ROTATE = 3
    REMOVER = 4
    SIZE = 5

class ClassStatus(Enum):
    CAR = 0
    TRUCK = 1
    PEDESTRIAN = 2
    MOTORBIKE = 3
    BICYCLE = 4
    NONE = 5
#-------------------------------------------------------#

#------------------- MAIN FLOW --------------------------#
class MainWindow():

    #----------------
    def __init__(self, main):
        #TODO 可能であれば1280へ変更
        self.width = 1560
        #self.width = 640
        self.height = 640
        self.draw_line_status = DrawStatus.IDLE
        self.draw_area_status = DrawStatus.IDLE

        self.annotations = dict()
        self.matched_objects = dict()
        self.objects = dict()
        self.class_status = ClassStatus.NONE

        #TODO temoorary SVC220 system as an only one sensor
        ###
        # self.objects = {
        #                 "camera":[].append({"id", "pos"})
        #                 "radar":[].append({"id", "pos"})
        #                 "sonar":[].append({"id", "pos"})
        # }
        #
        # self.matched_objects = {id:[].append({time_stamp, annot_id, sensor_id, obj_id, ref_x, ref_y, tar_x, tar_y})}
        # }
        #when you save, modify dictionary or delete objects id from self.objects << great selection

        self.selected_ano_key = ''
        self.ano_count = 0

        # ---- save setting ----#
        #self.f = open(file_name, 'w', newline='')
        with open(file_name, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["timer", "tar_x", "tar_y", "tar_w", "tar_l", "tar_theta", "tar_class", "status", "obs_x", "obs_y","obs_vx","obs_vy" "obs_w", "obs_l", "obs_theta", "obs_class"])

        #--------call fusion viewer ----------#

        #TODO add objects information to reverse variables
        self.Loader = Loader(args)
        #self.lidar_image, self.obs_data, self.end_flag, self.timer, frame = self.Loader.move_to_start_position()
        self.lidar_image, self.obs_data, self.end_flag, self.timer = self.Loader.move_to_start_position()

        #cv2.imwrite("sample_camera.jpg", frame)

        #print("tksize", self.lidar_image)
        #TODO modify here for next step
        self.step_width = 0.2

        #---------------------------------#
        

        #------- Create canvas for  visualioze returned ploting-image-----#
        self.canvas = Canvas(main, width=self.width, height=self.height)
        self.canvas.config(cursor='none')
        self.canvas.grid(row=0, column=0, columnspan=6, rowspan=30)

        #-----------------------------------------------------------------#


        self.classes = ["car", "pedestrian", "bicycle","Truck", "motorbike", "None"]
        self.class_colors = ["red", "cyan", "green", "purple", "orange", "yellow"]

        #------- translate format nd add image to canvas-----#
        #TODO プロット画像のスケール感を明記する
        #self.lidar_image[scale] = np.concatenate([self.lidar_image[scale], self.lidar_image[scale]],axis=1)
        #print("shape:",self.lidar_image[scale].shape)
        
        self.lidar_image = cv2totk(self.lidar_image)
        self.image_on_canvas = self.canvas.create_image(
            0, 0, anchor=NW, image=self.lidar_image)

        #TODO show all objects
        #print("obs data:", self.obs_data)
        self.showAll()

        # 各種ボタンを作る
        self.class_buttons = []
        for i in range(len(self.classes)):
            column = 14
            row = i + 20 
            class_name = self.classes[i]
            class_color = self.class_colors[i]
            class_button = HoverButton(main, text=class_name, bg=class_color, activebackground=class_color)#, width=25)
            class_button.bind("<Button-1>", self.onmakeButton)
            class_button.grid(row=row, column=column, columnspan=1, rowspan=1, sticky='nwse')
            self.class_buttons.append(class_button)


        column = 12

        # 次の画像を表示するボタン
        self.button_next = Button(
            main, text="Next", command=self.onNextButton)#, width=20, height=5)
        self.button_next.grid(row=31, column=1, columnspan=1, rowspan=1, sticky='nwse')
        # 選択領域を保存するボタン
        self.button_save = Button(
            main, text="Save", command=self.onSaveButton)#, width=25, height=5)
        self.button_save.grid(row=21, column=column + 1, columnspan=1, rowspan=1, sticky='nwse')
        #delete taeget class object
        self.button_delete = Button(
            main, text="Delete", command=self.onDeleteAnnotation)#, width=25, height=5)
        self.button_delete.grid(row=20, column=column + 1, columnspan=1, rowspan=1, sticky='nwse')
        # skip button
        self.button_skip = Button(
            main, text="Skip", command=self.onSkipButton)#, width=25, height=5)
        self.button_skip.grid(row=31, column=4, columnspan=1, rowspan=1, sticky='nwse')
        #Finish buton
        self.button_finish = Button(
            main, text="Finish", command=self.onFinishButton)#, width=25, height=5)
        self.button_finish.grid(row=23, column=column+1, columnspan=1, rowspan=1, sticky='nwse')

        #TODO Dlete
        #Make animation button
        self.Animator_button = Button(
            main, text="Animator", command=self.onAnimator)#, width=25, height=5)
        self.Animator_button.grid(row=24, column=column+1, columnspan=1, rowspan=1, sticky='nwse')

        # Annotation button
        self.button_make = Button(
            main, text="make object", command=self.onmakeButton)#, width=25, height=5)
        self.button_make.grid(row=22, column=column + 1, columnspan=1, rowspan=1, sticky='nwse')

 
        #Set List-box
        var = StringVar(value=[])
        self.listbox = Listbox(main,
                     listvariable=var)
        #self.listbox.grid(row=0, column=column + 1, columnspan=1, rowspan=31, sticky='nwse')
        self.listbox.grid(row=0, column=column + 1, columnspan=1, rowspan=20, sticky='nwse')
        self.listbox.bind("<<ListboxSelect>>", self.onListSelected)

        #Set Lael frame for tageting sensors
        self.sense_frame = LabelFrame(main, text="Target sensors")
        self.sense_frame.grid(row=0, column=column+2, padx=1, pady=20, sticky="e")
        self.sensor_selection = {}

        #Set Label for animation generator
        self.animator_frame = LabelFrame(main, text="Animator set-up")
        self.animator_frame.grid(row=1, column=column+2, padx=2, pady=20, sticky="e")
        Label(self.animator_frame, text="start:", justify='left').pack()
        Entry(self.animator_frame, width=10, justify='right').pack()
        Label(self.animator_frame, text="End:").pack()
        Entry(self.animator_frame, width=10, justify='right').pack()
        Label(self.animator_frame, text="FileName:").pack()
        Entry(self.animator_frame, width=10, justify='left').pack()
        
        #Make check button
        self.target_sensors = {}
        self.target_sensors["SRR220_Front"] = None
        for key in sensor_list:
            sensor_list[key] = StringVar(value=False)
            Checkbutton(self.sense_frame, variable=sensor_list[key],text=key).pack()

        self.txt_img_num = Entry(width=5, justify='center')
        self.txt_img_num.insert(END, '1')
        self.txt_img_num.grid(row=31, column=2, columnspan=1, rowspan=1, sticky='nwse')

        # スライダ用の変数を確保
        self.mouse_down = False
        self.fix_x = -1
        self.fix_y = -1

        # Add MouseEvent
        self.canvas.bind("<Button-1>", self.onMouseDown)
        self.canvas.bind("<Button-3>", self.onMouseDownRight)
        self.canvas.bind("<ButtonRelease-1>", self.onMouseUp)
        self.canvas.bind("<ButtonRelease-3>", self.onMouseUp)
        self.canvas.bind('<Motion>', self.onMotion)
        self.canvas.bind('<Leave>', self.onLeave)

        #Iinitialize input status and annotaion status
        self.input_status = InputStatus.SELECT
        self.rect = None
        self.annot_ = None
        self.annot = None
        self.scale = 1

        #temporal box condition
        self.line_head_x = None
        self.line_head_y = None
        self.line_tail_x = None
        self.line_tail_y = None
        self.line_side_x = None
        self.line_side_y = None

        #temp box ids for annoting box
        debug_mode = True
        if debug_mode:
            self.time_stamp = 0
            self.obj_id = 0

    #----------------
    def Clear(self):
        self.draw_line_status = DrawStatus.IDLE
        #TODO delete annotations and add objects data
        self.annotations = dict() #leave here for the first phase
        self.matched_objects = dict()
        self.selected_ano_key = ''
        self.ano_count = 0
        self.listbox.delete(0, END)
        #delete line
        self.ClearNowAnoPosition()
    
    #TODO read the objedata and add them to listbox, and clear all object's data and insert new one
    #needless
    #def _read_objdata(self):
    #    obj_data = self.objects
    #    for obj_id in self.objects:
    #        self.selected_ano_key = "obj_" + obj_id
    #        self.listbox.insert(END, self.selected_ano_key)
    #        self.listbox.selection_clear(0, END)
    #        self.listbox.selection_set(END)
    #    
    #    #self._drawline()
    #    self._allbox()

    def _drawline(self):
        if self.selected_ano_key != '':
            ano = self.annotations[self.selected_ano_key]
            #ano = self.annotations[self.selected_obj_key]
            selected_obj_name = self.objects[self.selected_obj_key].name

            self.canvas.create_text(ano.left_top_x, ano.left_top_y, text=selected_obj_name, fill=self.class_colors[selected_obj_key]
                                    , justify="left", tag="name", anchor="sw")
            self._draw_box_withline(ano)

    
    def DisactiveControler(self):
        #self.button_back.config(state="disable")
        self.button_next.config(state="disable")
        self.button_save.config(state="disable")
        self.button_delete.config(state="disable")
        self.button_skip.config(state="disable")
        self.button_finish.config(state="disable")
        self.button_make.config(state="disable")
        #self.button_move.config(state="disable")
        self.listbox.config(state="disable")
        self.txt_img_num.config(state="disable")
        for btn in self.class_buttons:
                btn.config(state="disable")
    
    def ActiveControler(self):
        #self.button_back.config(state="normal")
        self.button_next.config(state="normal")
        self.button_save.config(state="normal")
        self.button_delete.config(state="normal")
        self.button_skip.config(state="normal")
        self.button_finish.config(state="normal")
        self.listbox.config(state="normal")
        self.txt_img_num.config(state="normal")
        self.button_make.config(state="normal")
        for btn in self.class_buttons:
                btn.config(state="normal")


    def onNextButton(self):
        """TODO list in this function
        1. call self.main_fusion's step_forward function
        2. get following varibles 
           1.lidar image
           2.cameras images
           3.object data
           4.
        """

        #---- detect ----#
        for ano_key in self.annotations:
            ano = self.annotations[ano_key]
            for s_id in ano.matched_obs_data:
                for obs_id in ano.matched_obs_data[s_id]:
                    #print("obs_id", obs_id)
                    #TODO make list, and write here
                    timer = self.timer
                    tar_id, tar_x, tar_y, tar_w, tar_l, tar_theta, tar_class = ano.obj_id, (ano.head_x-center_u)/10, (ano.head_y-center_v)/10, ano.w_/10, ano.l_/10, ano.theta_, ano.obj_class
                    status = "detect"
                    obj = self.obs_data[s_id][obs_id]
                    obs_x, obs_y, obs_vx, obs_vy, obs_w, obs_l, obs_theta, obs_class = obj.x, obj.y, obj.vx, obj.vy, obj.w, obj.l, obj.o, obj.cls
                    sense_id = s_id
                    datalist = [self.timer, tar_id, tar_x, tar_y, tar_w, tar_l, tar_theta, tar_class, status, sense_id, obs_x, obs_y, obs_vx, obs_vy, obs_w, obs_l, obs_theta, obs_class]
                    #print("data1:", datalist)
                    with open(file_name, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(datalist)

        #---- missdetect ----#
        print("obs_data:", self.obs_data)
        for s_id in self.obs_data:
            if s_id !="LiDAR" and s_id != "IMX490" and s_id != "VehicleCAN":
                for obj in self.obs_data[s_id]:
                    print(s_id)

                    if obj.match == False:
                        #TODO make list, and write here
                        tar_id, tar_x, tar_y, tar_w, tar_l, tar_theta, tar_class = None, 0, 0, 0, 0, 0, 0
                        status = "missdetect"
                        #obs_x, obs_y, obs_w, obs_l, obs_theta, obs_class = obj.x, obj.y, obj.w, obj.l, obj.o, obj.cls
                        obs_x, obs_y, obs_vx, obs_vy, obs_w, obs_l, obs_theta, obs_class = obj.x, obj.y, obj.vx, obj.vy, obj.w, obj.l, obj.o, obj.cls
                        #obs_x, obs_y, obs_w, obs_l, obs_theta, obs_class = obj.pos[0], -obj.pos[1], obj.shape[0], obj.shape[1], -obj.orientation, obj.obj_class
                        #datalist = [self.timer, tar_id, tar_x, tar_y, tar_w, tar_l, tar_theta, tar_class, status, obs_x, obs_y, obs_w, obs_l, obs_theta, obs_class]
                        sense_id = s_id
                        #datalist = [self.timer, tar_id, tar_x, tar_y, tar_w, tar_l, tar_theta, tar_class, status, sense_id, obs_x, obs_y, obs_w, obs_l, obs_theta, obs_class]
                        datalist = [self.timer, tar_id, tar_x, tar_y, tar_w, tar_l, tar_theta, tar_class, status, sense_id, obs_x, obs_y,obs_vx, obs_vy, obs_w, obs_l, obs_theta, obs_class]
                        #print("data2:", datalist)
                        with open(file_name, 'a', newline='') as f:
                            writer = csv.writer(f)
                            writer.writerow(datalist)

        #---- undetect ----#
        for ano_key in self.annotations:
            ano = self.annotations[ano_key]
            for s_id in self.obs_data:
                if s_id !="LiDAR" and s_id != "IMX490" and s_id != "VehicleCAN":
                    if len(ano.matched_obs_data[s_id]) == 0:
                        #TODO make list, and write here
                        timer = self.timer
                        tar_id, tar_x, tar_y, tar_w, tar_l, tar_theta, tar_class = ano.obj_id, (ano.head_x-center_u)/10, (ano.head_y-center_v)/10, ano.w_/10, ano.l_/10, ano.theta_, ano.obj_class
                        status = "undetect"
                        obs_x, obs_y, obs_vx, obs_vy, obs_w, obs_l, obs_theta, obs_class = 0, 0, 0, 0, 0, 0, 0, 0
                        sense_id = s_id
                        #datalist = [self.timer, tar_id, tar_x, tar_y, tar_w, tar_l, tar_theta, tar_class, status, sense_id, obs_x, obs_y, obs_w, obs_l, obs_theta, obs_class]
                        datalist = [self.timer, tar_id, tar_x, tar_y, tar_w, tar_l, tar_theta, tar_class, status, sense_id, obs_x, obs_y,obs_vx, obs_vy, obs_w, obs_l, obs_theta, obs_class]
                        #print("data3:", datalist)
                        with open(file_name, 'a', newline='') as f:
                            writer = csv.writer(f)
                            writer.writerow(datalist)


        #self.matched_obs_data = []
        for ano_key in self.annotations:
            ano = self.annotations[ano_key]
            ano.matched_obs_data = {}
            for s_id in sensor_color:
                ano.matched_obs_data[s_id] = []

        self.deleteAll()
        #self.canvas.delete("all")

        self.lidar_image, self.obs_data, self.end_flag, self.timer = self.Loader.move_to_next_position(self.step_width)
        #self.lidar_image, self.obs_data, self.end_flag, self.timer, frame = self.Loader.move_to_next_position(self.step_width)
        
        self.showAll()

        name_cam = "./image/scene_"+str(timer)+".jpg"
        name_plot = "./image/scene_"+str(timer)+"_lidar.jpg"
        #cv2.imwrite(name_cam, frame)
        cv2.imwrite(name_plot, self.lidar_image)

        self.lidar_image = cv2totk(self.lidar_image)
        self.canvas.itemconfig(self.image_on_canvas,
                               image=self.lidar_image)
        #print(self.lidar_image[scale-1])

        #return lidar_image, sur_data, tmp_object_data, end_flag, timer

        


    
    
    def onSkipButton(self):
        # 一つ進む
        num = int(self.txt_img_num.get())
        if num > len(self.my_images) or num < 1:
            raise Exception('out of image number')

        self.my_image_number = num - 1

        self.lidar_image, self.sur_cam_image, self.obs_data, self.end_flag, self.timer, frame = self.Loader.step_forward()

        # 表示画像を更新
        if not self.end_flag:
            self.canvas.itemconfig(self.image_on_canvas,
                                   image=self.lidar_image)
        else:
            sys.exit()

        # 表示画像にあわせてラインの位置を更新

        # labelを更新
        #TODO replace filename to timer 
        self.img_file_name_txt.set(str(self.timer))
        
        self.Clear()
        self.LoadAnoData()
        #self._showall()

    def onSaveButton(self):
        self._save()
    
    #def onShowAllButton(self):
    #    self.ClearNowAnoPosition()
    #    self._showall()
    
    #TODO remake showall -> not only annnotation data but also the target object
    #def _showall(self):
    #    pass
    #    for key in self.annotations:
    #        ano = self.annotations[key]
    #        selected_class_name = ano.class_name
    #        self.canvas.create_rectangle(ano.left_top_x, ano.left_top_y, ano.right_bottom_x, ano.right_bottom_y
    #                                , tag="bbox", outline=self.class_colors[selected_class_name])
    #        self.canvas.create_text(ano.left_top_x, ano.left_top_y, text=selected_class_name, fill=self.class_colors[selected_class_name]
    #                                , justify="left", tag="name", anchor="sw")

    def _save(self):
        #TODO
        #1. loop matched obs data
        #2.  
        file_name = str(self.timer) + '.txt'
        print('save annotation file : ' + file_name)
        with (ano_dir / file_name).open(mode='w', encoding='utf-8') as f:
        #with open(ano_dir / file + ".csv", mode="a", newline="") as f:
            #for key in self.annotations:
            for key in self.objects:
                ano = self.objects[key]
                ano.left_top_x = np.clip(ano.left_top_x, 0, self.width)
                ano.right_bottom_x = np.clip(ano.right_bottom_x, 0, self.width)
                ano.left_top_y = np.clip(ano.left_top_y, 0, self.height)
                ano.right_bottom_y = np.clip(ano.right_bottom_y, 0, self.height)
                f.write(ano.class_name + '\t' + str(ano.left_top_x) + '\t' + str(ano.left_top_y) + '\t' + str(ano.right_bottom_x) + '\t' + str(ano.right_bottom_y) + '\n')

    #TODO delete annotation but do not delete target name
    def onDeleteAnnotation(self):
        if len(self.listbox.curselection()) == 0:
            return
        index = self.listbox.curselection()[0]
        self.selected_ano_key = self.listbox.get(index)

        self.canvas.delete(self.annotations[self.selected_ano_key].vis_id)
        self.canvas.delete(self.annotations[self.selected_ano_key].vis_head)
        del self.annotations[self.selected_ano_key]
        self.listbox.delete(index)

        self.selected_ano_key = ''

    def onLeave(self, event):
        self.canvas.delete("xy")
        self.canvas.delete("suport_line")
        self._mouseup()

    def onMotion(self, event):
        line0 = 'u:' + str(event.x) + ' ' + 'v:' + str(event.y)
        #TODO change here
        #line1 = 'x:' + str((-event.y+center_u)/vis_scale) + 'm ' + 'y:' + str((event.x-center_v)/vis_scale) + 'm'
        line1 = 'x:' + str((event.x-center_u)/vis_scale) + 'm ' + 'y:' + str((-event.y+center_v)/vis_scale) + 'm'
        self.canvas.delete("xy")
        self.canvas.delete("suport_line")
        self.canvas.create_text(100, 10, text=line0, tag="xy", font=10, fill="green", justify="left")
        self.canvas.create_text(100, 30, text=line1, tag="xy", font=10, fill="green", justify="left")

        if self.mouse_down:
            ## -- Select mode -- ##
            if self.input_status is InputStatus.SELECT:
                if self.draw_area_status == DrawStatus.DRAWING:
                    self.visualize_selected_area(event, color="yellow")
            ## -- Remove mode -- ##
            elif self.input_status is InputStatus.REMOVER:
                if self.draw_area_status == DrawStatus.DRAWING:
                    self.visualize_selected_area(event, color="red")
            ## -- Move mode -- ##
            elif self.input_status is InputStatus.MOVE:
                self.move_box(event)
            ## -- Rotate mode -- ##
            elif self.input_status is InputStatus.ROTATE:
                self.rotate_box(event)
            ## -- Sizing mode -- ##
            elif self.input_status is InputStatus.SIZE:
                self.size_box(event)

        else:
            self.canvas.create_line(0, event.y, event.x - 5, event.y, tag="suport_line", fill='white')
            self.canvas.create_line(event.x + 5, event.y, self.width, event.y, tag="suport_line", fill='white')
            
            self.canvas.create_line(event.x, 0, event.x, event.y - 5, tag="suport_line", fill='white')
            self.canvas.create_line(event.x, event.y + 5, event.x, self.height, tag="suport_line", fill='white')        

            self.canvas.create_line(event.x, event.y, event.x + 1, event.y, tag="suport_line", fill='white')


            if self.draw_line_status is DrawStatus.IDLE:
                if self.selected_ano_key != '':
                    #print("ano_key",self.selected_ano_key)
                    ano = self.annotations[self.selected_ano_key]
                    #Move judgenment
                    wk_x_df = True if ano.head_x - 4 < event.x and ano.head_x + 4 > event.x else False
                    wk_y_df = True if ano.head_y - 4 < event.y and ano.head_y + 4 > event.y else False

                    #Rotate judgement
                    wk_x_rt = True if ano.tail_x - 4 < event.x and ano.tail_x + 4 > event.x else False
                    wk_y_rt = True if ano.tail_y - 4 < event.y and ano.tail_y + 4 > event.y else False

                    #Sizing judgement
                    df_x, df_y = 2*(ano.x_ - ano.head_x), 2*(ano.y_ - ano.head_y)
                    #for tail
                    wk_x_tl = True if ano.tail_x + df_x - 4 < event.x and ano.tail_x + df_x + 4 > event.x else False
                    wk_y_tl = True if ano.tail_y + df_y - 4 < event.y and ano.tail_y  + df_y + 4 > event.y else False
                    #for head
                    wk_x_hd = True if ano.head_x + df_x - 4 < event.x and ano.head_x + df_x + 4 > event.x else False
                    wk_y_hd = True if ano.head_y + df_y - 4 < event.y and ano.head_y  + df_y + 4 > event.y else False

                    if (wk_x_df and wk_y_df):
                        self.canvas.config(cursor='hand1')
                    elif (wk_x_rt and wk_y_rt):
                        self.canvas.config(cursor='exchange')
                    elif (wk_x_tl and wk_y_tl):
                        self.canvas.config(cursor='sizing')
                    elif (wk_x_hd and wk_y_hd):
                        self.canvas.config(cursor='sizing')
                    else:
                        self.canvas.config(cursor='none')
            
            #TODO make annotatin
            elif self.draw_line_status is DrawStatus.TAIL:
                self.line_tail_x = event.x
                self.line_tail_y = event.y
                self.visualize_annotation_box()
            elif self.draw_line_status is DrawStatus.SIDE:
                self.line_side_x = event.x
                self.line_side_y = event.y
                self.visualize_annotation_box()


    def onMouseDown(self, event):
        self.mouse_down = True
        #TODO branching processes depending on input status
        if self.selected_ano_key != '':
            ano = self.annotations[self.selected_ano_key]

            #move judgement 
            wk_x_df = True if ano.head_x - 4 < event.x and ano.head_x + 4 > event.x else False
            wk_y_df = True if ano.head_y - 4 < event.y and ano.head_y + 4 > event.y else False

            #Rotate judgement
            wk_x_rt = True if ano.tail_x - 4 < event.x and ano.tail_x + 4 > event.x else False
            wk_y_rt = True if ano.tail_y - 4 < event.y and ano.tail_y + 4 > event.y else False

            #Sizing judgement
            df_x, df_y = 2*(ano.x_ - ano.head_x), 2*(ano.y_ - ano.head_y)
            #for tail
            wk_x_tl = True if ano.tail_x + df_x - 4 < event.x and ano.tail_x + df_x + 4 > event.x else False
            wk_y_tl = True if ano.tail_y + df_y - 4 < event.y and ano.tail_y  + df_y + 4 > event.y else False
            #for head
            wk_x_hd = True if ano.head_x + df_x - 4 < event.x and ano.head_x + df_x + 4 > event.x else False
            wk_y_hd = True if ano.head_y + df_y - 4 < event.y and ano.head_y  + df_y + 4 > event.y else False

 
            ####----  Move mode ----####
            if wk_x_df and wk_y_df:
                self.input_status = InputStatus.MOVE
            ####----  Rotate mode ----####
            elif wk_x_rt and wk_y_rt:
                self.input_status = InputStatus.ROTATE
            ####----  Sizing mode ----####
            elif wk_x_tl and wk_y_tl:
                self.input_status = InputStatus.SIZE
                self.draw_line_status = DrawStatus.REAR
            elif wk_x_hd and wk_y_hd:
                self.input_status = InputStatus.SIZE
                self.draw_line_status = DrawStatus.FRONT

        if self.input_status == InputStatus.SELECT:
            if self.draw_area_status == DrawStatus.IDLE:
                self.rect_start_x = event.x
                self.rect_start_y = event.y
                self.draw_area_status = DrawStatus.DRAWING

        elif self.input_status == InputStatus.MAKE:
            if self.draw_line_status is DrawStatus.IDLE:
                self.line_head_x = event.x
                self.line_head_y = event.y
                self.draw_line_status = DrawStatus.TAIL

            elif self.draw_line_status is DrawStatus.TAIL:
                self.line_tail_x = event.x
                self.line_tail_y = event.y
                self.draw_line_status = DrawStatus.SIDE

            elif self.draw_line_status is DrawStatus.SIDE:
                self.line_side_x = event.x
                self.line_side_y = event.y
                self.draw_line_status = DrawStatus.IDLE

                #finish annotate target object
                self.add_box_on_annotations()
                self.input_status = InputStatus.SELECT
                self.visualize_annotation_box()
                self.ActiveControler()

        #elif self.input_status is InputStatus.MOVE:
        #    if self.selected_ano_key != '':
        #        ano = self.annotations[self.selected_ano_key]

        #        wk_x_df = True if ano.head_x - 4 < event.x and ano.head_x + 4 > event.x else False
        #        wk_y_df = True if ano.head_y - 4 < event.y and ano.head_y + 4 > event.y else False

        #        if wk_x_df and wk_y_df:
        #            self.input_status = InputStatus.MOVE
    
    def onMouseDownRight(self, event):
        self.mouse_down = True

        if self.input_status == InputStatus.SELECT:
            if self.draw_area_status == DrawStatus.IDLE:
                self.rect_start_x = event.x
                self.rect_start_y = event.y
                self.input_status = InputStatus.REMOVER
                self.draw_area_status = DrawStatus.DRAWING

    def onMouseUp(self, event):
        self._mouseup()
        #self._showall()
    
    def _mouseup(self):
        self.mouse_down = False
        #TODO branch processing  with input status
        if self.input_status is InputStatus.SELECT:
            
            if self.draw_area_status == DrawStatus.DRAWING and self.selected_ano_key is not None:
                ####---code around here to moveand on mouse up function ----#####
                for s_id in self.obs_data:
                    if s_id != "LiDAR" and s_id != "IMX490" and s_id != "VehicleCAN":
                        for obs_id in range(len(self.obs_data[s_id])):
                            self.canvas.delete(self.obs_data[s_id][obs_id])

                            obj = self.obs_data[s_id][obs_id]
                            ux = obj.x*vis_scale + center_u
                            uy = -obj.y*vis_scale + center_v
                            start_x = min(self.rect_start_x, self.rect_end_x)
                            start_y = min(self.rect_start_y, self.rect_end_y)
                            end_x = max(self.rect_start_x, self.rect_end_x)
                            end_y = max(self.rect_start_y, self.rect_end_y)
                            if  ( start_x < ux < end_x and start_y < uy < end_y):
                                #add to annotation objects
                                judge = obs_id in self.annotations[self.selected_ano_key].matched_obs_data[s_id]
                                if not judge:
                                    self.annotations[self.selected_ano_key].matched_obs_data[s_id].append(obs_id)

                                #change status "match" to True
                                obj.match = True

                                #rvisualize obs data
                                self.canvas.delete(self.obs_data[s_id][obs_id].img)
                                self.obs_data[s_id][obs_id].img = self.showbox(s_id, obs_id, color="red")


            self.canvas.delete(self.rect)
            self.rect = None
            self.input_status = InputStatus.SELECT
            self.draw_area_status = DrawStatus.IDLE
            ##################################

        elif self.input_status is InputStatus.REMOVER:
            ####---code around here to moveand on mouse up function ----#####
            if self.draw_area_status == DrawStatus.DRAWING:
                for s_id in self.obs_data:
                    if s_id != "LiDAR" and s_id != "IMX490" and s_id != "VehicleCAN":
                        for obs_id in range(len(self.obs_data[s_id])):

                            obj = self.obs_data[s_id][obs_id]
                            ux = obj.x*vis_scale + center_u
                            uy = -obj.y*vis_scale + center_v
                            start_x = min(self.rect_start_x, self.rect_end_x)
                            start_y = min(self.rect_start_y, self.rect_end_y)
                            end_x = max(self.rect_start_x, self.rect_end_x)
                            end_y = max(self.rect_start_y, self.rect_end_y)
                            if  ( start_x < ux < end_x and start_y < uy < end_y):
                                #add to annotation objects
                                #print("obsid:", obs_id)
                                judge = obs_id in self.annotations[self.selected_ano_key].matched_obs_data[s_id]
                                if judge:
                                    self.annotations[self.selected_ano_key].matched_obs_data[s_id].remove(obs_id)

                                #change status "match" to False
                                obj.match = False

                                #rvisualize obs data
                                self.canvas.delete(obj.img)
                                self.obs_data[s_id][obs_id].img = self.showbox(s_id, obs_id, color=sensor_color[s_id])

            self.canvas.delete(self.rect)
            self.rect = None
            self.input_status = InputStatus.SELECT
            self.draw_area_status = DrawStatus.IDLE
            ##################################

        elif self.input_status is InputStatus.MOVE:
            self.input_status = InputStatus.SELECT
        elif self.input_status is InputStatus.ROTATE:
            self.input_status = InputStatus.SELECT
        elif self.input_status is InputStatus.SIZE:
            self.input_status = InputStatus.SELECT
            self.draw_line_status = DrawStatus.IDLE

    def onListSelected(self, event):
        if len(self.listbox.curselection()) == 0:
            return
        index = self.listbox.curselection()[0]
        name = self.listbox.get(index)
        self.ClearNowAnoPosition()
        self.selected_ano_key = name
        #change color 
        for index in self.annotations:
            self.selected_ano_key = index
            self.canvas.delete(self.annotations[self.selected_ano_key].vis_id)
            self.canvas.delete(self.annotations[self.selected_ano_key].vis_head)
            self.visualize_annotation_box(target_color="white")
        self.selected_ano_key = name
        self.canvas.delete(self.annotations[self.selected_ano_key].vis_id)
        self.canvas.delete(self.annotations[self.selected_ano_key].vis_head)
        self.visualize_annotation_box(target_color="green")


    def ClearNowAnoPosition(self):
        self.canvas.delete("bbox")
        self.canvas.delete("name")

    
    def onFinishButton(self):
        sys.exit()


    def onmakeButton(self, event):
        self.input_status = InputStatus.MAKE
        if self.selected_ano_key != '':
            self.canvas.delete(self.annotations[self.selected_ano_key].vis_id)
            self.canvas.delete(self.annotations[self.selected_ano_key].vis_head)
            self.visualize_annotation_box("white")
        self.selected_ano_key = "obj_" + str(self.obj_id)
        self.annotations[self.selected_ano_key] = Annotation(self.obj_id, self.timer) #annotate without class
        #TODO add class id
        target_class = event.widget["text"]
        self.annotations[self.selected_ano_key].obj_class = target_class
        #print(target_class)
        self.class_status = ClassStatus.NONE
        self.obj_id += 1

        self.listbox.insert(END, self.selected_ano_key)
        self.listbox.selection_clear(0, END)
        self.listbox.selection_set(END)

        self.DisactiveControler()


    #TODO hardest point, but use sur_came_config module and transform uv coordinate to lidar coordinate
    def transformcoordinate(self, status):
        pass


    def move_box(self, event):
        ano = self.annotations[self.selected_ano_key]
        x = event.x
        y = event.y
        diff_x = x - ano.head_x
        diff_y = y - ano.head_y
        #move vis and vis_head
        self.canvas.move(ano.vis_id, diff_x, diff_y)
        self.canvas.move(ano.vis_head, diff_x, diff_y)
        self.move_status(diff_x, diff_y)


    def rotate_box(self, event):
        ano = self.annotations[self.selected_ano_key]
        rotated = []

        func = lambda theta, diff: np.array([[cos(theta), -sin(theta)],[sin(theta), cos(theta)]]) @ diff + np.array([ano.c_x, -ano.c_y]) 

        theta_after = arctan2(-event.y + ano.c_y, event.x - ano.c_x)
        theta_pre = arctan2(-ano.tail_y + ano.c_y, ano.tail_x - ano.c_x)

        for con in [[ano.head_x, ano.head_y], [ano.tail_x, ano.tail_y], [ano.side_x, ano.side_y], [ano.x_, ano.y_]]:
            diff = np.array([con[0] - ano.c_x, -con[1] + ano.c_y])
            con_ = func(theta_after - theta_pre, diff)
            con_[1] = -con_[1]
            rotated.append(con_)
        
        #reset and 
        self.rotate_status(rotated)

        #visualize
        self.canvas.delete(ano.vis_id)
        self.canvas.delete(ano.vis_head)
        self.visualize_annotation_box()
     
    def size_box(self, event):
        ano = self.annotations[self.selected_ano_key]

        ano.side_x = event.x
        ano.side_y = event.y

        if self.draw_line_status == DrawStatus.REAR:
            con_x = ano.tail_x
            con_y = ano.tail_y
        elif self.draw_line_status == DrawStatus.FRONT:
            con_x = ano.head_x
            con_y = ano.head_y

        diff_x, diff_y, d, theta = self.get_diff_xy(ano.head_x, ano.head_y, ano.tail_x, ano.tail_y, ano.side_x, ano.side_y)
        tail_x = con_x + diff_x
        tail_y = con_y + diff_y
        diff = [event.x - tail_x, tail_y - event.y]
        alpha = arctan2(diff[1], diff[0])
        r = sqrt((event.x - tail_x)**2.0 + (event.y - tail_y)**2.0)
        diff = [r*cos(theta+pi-alpha)*cos(theta+pi), -r*cos(theta+pi-alpha)*sin(theta+pi)]
        
        if self.draw_line_status == DrawStatus.REAR:
            ano.tail_x += diff[0]
            ano.tail_y += diff[1]
        elif self.draw_line_status == DrawStatus.FRONT:
            ano.head_x += diff[0]
            ano.head_y += diff[1]


        #visualize
        self.canvas.delete(ano.vis_id)
        self.canvas.delete(ano.vis_head)
        self.visualize_annotation_box()


    def move_status(self, diff_x, diff_y):
        ano = self.annotations[self.selected_ano_key]
        #annotation status
        ano.head_x = ano.head_x + diff_x
        ano.head_y = ano.head_y + diff_y
        ano.side_x = ano.side_x + diff_x
        ano.side_y = ano.side_y + diff_y
        ano.tail_x = ano.tail_x + diff_x
        ano.tail_y = ano.tail_y + diff_y
        #head status
        ano.x_ = ano.x_ + diff_x
        ano.y_ = ano.y_ + diff_y
        #center pos
        ano.c_x = ano.c_x + diff_x
        ano.c_y = ano.c_y + diff_y
    
    def rotate_status(self, rotated):
        ano = self.annotations[self.selected_ano_key]
        ano.head_x = rotated[0][0]
        ano.head_y = rotated[0][1]
        ano.tail_x = rotated[1][0]
        ano.tail_y = rotated[1][1]
        ano.side_x = rotated[2][0]
        ano.side_y = rotated[2][1]
        ano.x_ = rotated[3][0] 
        ano.y_ = rotated[3][1] 

    def add_box_on_annotations(self):
        ano = self.annotations[self.selected_ano_key]
        ano.head_x = self.line_head_x 
        ano.head_y = self.line_head_y
        ano.side_x = self.line_side_x
        ano.side_y = self.line_side_y
        ano.tail_x = self.line_tail_x
        ano.tail_y = self.line_tail_y


    def visualize_selected_area(self, event, color):
        self.rect_end_x = event.x
        self.rect_end_y = event.y

        if self.rect:
            #coords redefines rectangle 
            self.canvas.coords(self.rect,
                min(self.rect_start_x, self.rect_end_x),
                min(self.rect_start_y, self.rect_end_y),
                max(self.rect_start_x, self.rect_end_x),
                max(self.rect_start_y, self.rect_end_y)
                )
        else:
            self.rect = self.canvas.create_rectangle(
                min(self.rect_start_x, self.rect_end_x),
                min(self.rect_start_y, self.rect_end_y),
                max(self.rect_start_x, self.rect_end_x),
                max(self.rect_start_y, self.rect_end_y), 
                outline='yellow', fill=color, stipple="gray50"
                )

    
    def visualize_annotation_box(self, target_color="green"):

        if self.draw_line_status is DrawStatus.TAIL:
            if self.annot_:
                self.canvas.coords(self.annot_,
                    self.line_head_x, self.line_head_y,
                    self.line_tail_x, self.line_tail_y,
                    )
            else:
                self.annot_ = self.canvas.create_line(
                    self.line_head_x, self.line_head_y,
                    self.line_tail_x, self.line_tail_y,
                    fill='green'#, fill='blue', stipple="gray50"
                    )

        elif self.draw_line_status is DrawStatus.SIDE:

            diff_x, diff_y, d, theta = self.get_diff_xy(self.line_head_x, self.line_head_y, self.line_tail_x, self.line_tail_y, self.line_side_x, self.line_side_y)
            
            if self.annot:
                self.canvas.coords(self.annot,
                    #head -> side1 -> tail -> side2 -> head
                    self.line_head_x + diff_x, self.line_head_y + diff_y,
                    self.line_head_x, self.line_head_y,
                    self.line_tail_x, self.line_tail_y,
                    self.line_tail_x + diff_x, self.line_tail_y + diff_y,
                    self.line_head_x + diff_x, self.line_head_y + diff_y
                    )

            else:
                self.canvas.delete(self.annot_)
                self.annot_ = None
                self.annot = self.canvas.create_line(
                    #head -> side1 -> tail -> side2 -> head
                    self.line_head_x + diff_x, self.line_head_y + diff_y,
                    self.line_head_x, self.line_head_y,
                    self.line_tail_x, self.line_tail_y,
                    self.line_tail_x + diff_x, self.line_tail_y + diff_y,
                    self.line_head_x + diff_x, self.line_head_y + diff_y,
                    fill='green'#, fill='blue', stipple="gray50"
                    )


        else:
        #elif self.draw_line_status is DrawStatus.IDLE:
            ano = self.annotations[self.selected_ano_key]

            diff_x, diff_y, d, theta = self.get_diff_xy(ano.head_x, ano.head_y, ano.tail_x, ano.tail_y, ano.side_x, ano.side_y)

            self.canvas.delete(self.annot)
            self.annot = None

            ano.vis_id = self.canvas.create_line(
                #head -> side1 -> tail -> side2 -> head
                ano.head_x + diff_x, ano.head_y + diff_y,
                ano.head_x, ano.head_y,
                ano.tail_x, ano.tail_y,
                ano.tail_x + diff_x, ano.tail_y + diff_y,
                ano.head_x + diff_x, ano.head_y + diff_y,
                fill=target_color#, fill='blue', stipple="gray50"
                )

            ano.vis_head = self.canvas.create_oval(ano.head_x + diff_x//2 - 3, ano.head_y + diff_y//2 - 3,
                                    ano.head_x  + diff_x//2 + 3, ano.head_y + diff_y//2 + 3
                                    , fill="yellow") 
 
            ano.x_ = ano.head_x + diff_x//2
            ano.y_ = ano.head_y + diff_y//2
            ano.w_ = d
            ano.l_ = sqrt((ano.head_x - ano.tail_x)**2.0 + (ano.head_y - ano.tail_y)**2.0)
            ano.theta_ = theta
            ano.c_x = (ano.head_x + ano.tail_x) / 2 + diff_x /2
            ano.c_y = (ano.head_y + ano.tail_y) / 2 + diff_y /2

    def get_diff_xy(self, head_x, head_y, tail_x, tail_y, side_x, side_y):
        a = (head_y - tail_y) / (head_x - tail_x + 0.001)
        b = head_y - a * head_x
        d = abs(-a * side_x + side_y - b) / sqrt(a**2.0 + 1)
        theta = arctan2(-(head_y - tail_y), (head_x - tail_x))

        func = lambda x: a * x + b

        #xy -> uv therefore y's sign is inversed
        if 0 <= theta <= pi/2:
            alpha = theta
            if func(side_x) >= side_y:
                diff_x = -d * cos(pi/2 - alpha)
                diff_y = -d * sin(pi/2 - alpha)
            elif func(side_x) < side_y:
                diff_x = d * cos(pi/2 - alpha)
                diff_y = d * sin(pi/2 - alpha)
        elif pi/2 < theta <= pi:
            alpha = pi - theta
            if func(side_x) >= side_y:
                diff_x = d * cos(pi/2 - alpha)
                diff_y = -d * sin(pi/2 - alpha)
            elif func(side_x) < side_y:
                diff_x = -d * cos(pi/2 - alpha)
                diff_y = d * sin(pi/2 - alpha)
        elif -pi <= theta < -pi/2:
            alpha = abs(-pi - theta)
            if func(side_x) >= side_y:
                diff_x = -d * cos(pi/2 - alpha)
                diff_y = -d * sin(pi/2 - alpha)
            elif func(side_x) < side_y:
                diff_x = d * cos(pi/2 - alpha)
                diff_y = d * sin(pi/2 - alpha)
        elif -pi/2 <= theta < 0:
            alpha = abs(theta)
            if func(side_x) >= side_y:
                diff_x = d * cos(pi/2 - alpha)
                diff_y = -d * sin(pi/2 - alpha)
            elif func(side_x) < side_y:
                diff_x = -d * cos(pi/2 - alpha)
                diff_y = d * sin(pi/2 - alpha)
        return diff_x, diff_y, d, theta
    
    #Animation Generator
    def onAnimator(self):
        pass


    #-------drag scale---------#
    def dragScaler(self):
        pass

    def showAll(self):
        #for i in self.obs_data:
        #    ux = i.pos[0]*vis_scale + 640
        #    uy = -i.pos[1]*vis_scale + 320
        #    i.img = self.canvas.create_oval(ux - 5, uy - 5,
        #                                        ux + 5, uy + 5
        for s_id in self.obs_data:
            if s_id != "LiDAR" and s_id != "IMX490" and s_id != "VehicleCAN":
                for i in self.obs_data[s_id]:
                    ux = i.x*vis_scale + center_u
                    uy = -i.y*vis_scale + center_v
                    i.img = self.canvas.create_oval(ux - 3, uy - 3,
                                                        ux + 3, uy + 3
                                                   , fill=sensor_color[s_id]) 

    def showbox(self, s_id,obj_id, color="white"):
        #TODO change later
        #obj = self.obs_data[obj_id]
        #ux = obj.pos[0]*vis_scale + 640
        #uy = -obj.pos[1]*vis_scale + 320
        #obj.img = self.canvas.create_oval(ux - 5, uy - 5,
        #                                        ux + 5, uy + 5
        #                                       , fill=color)
        obj = self.obs_data[s_id][obj_id]
        ux = obj.x*vis_scale + center_u
        uy = -obj.y*vis_scale + center_v
        obj.img = self.canvas.create_oval(ux - 3, uy - 3,
                                                ux + 3, uy + 3
                                               , fill=color)
        return obj.img


    def deleteAll(self):
        for s_id in self.obs_data:
            if s_id !="LiDAR" and s_id != "IMX490" and s_id != "VehicleCAN":
                for i in self.obs_data[s_id]:
                    self.canvas.delete(i.img)
    #-------------------------#


class Annotation(object):
    def __init__(self, obj_id, time):
        self.obj_id = obj_id
        self.obj_class = None
        self.time_stamp = time
        self.vis_head = None
        #for world coordinate
        self.x_ = 0
        self.y_ = 0
        self.w_ = 0
        self.l_ = 0
        self.theta_ = 0
        #center_pos
        self.c_x = 0
        self.c_y = 0
        #tmp for annotation coordinate
        self.head_x = 0
        self.head_y = 0
        self.side_x = 0
        self.side_y = 0
        self.tail_x = 0
        self.tail_y = 0

        #matched obs_data
        self.matched_obs_data = {}
        for s_id in sensor_color:
            if s_id != "LiDAR" and s_id != "IMX490" and s_id != "VehicleCAN":
                self.matched_obs_data[s_id] = []
        

class HoverButton(Button):

    #class_button = HoverButton(main, text=class_name, bg=class_color, activebackground=class_color)#, width=25)
    def __init__(self, master, **kw):
        Button.__init__(self,master=master,**kw)
        self["relief"] = "groove"
        self.defaultBackground = self["bg"]
        self.bind("<Enter>", self.on_enter)
        self.bind("<Leave>", self.on_leave)

    def on_enter(self, e):
        self["bg"] = self["activebackground"]
        
        if self["state"] == "normal":
            self["bd"] = 2
        
    def on_leave(self, e):
        self["bg"] = self.defaultBackground
        self["bd"] = 1

#----------------------------------------------------------------------

root = Tk()
#main = Main(root)
MainWindow(root)
root.mainloop()


#2021/4/5 URl
#https://water2litter.net/rum/post/python_crop_objects/
#https://python.keicode.com/advanced/tkinter-widget-listbox.php
#https://stackoverflow.com/questions/54637795/how-to-make-a-tkinter-canvas-rectangle-transparent/54645103
#https://daeudaeu.com/tkinter_canvas_method/#moveto
#http://bacspot.dip.jp/virtual_link/www/si.musashi-tech.ac.jp/new_www/Python_IntroTkinter/03/index-5.html
#http://timiditybraver.blog71.fc2.com/blog-entry-354.html
#
