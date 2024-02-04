import sys
from PyQt5 import QtWidgets, uic, QtCore, QtGui
import cv2
import numpy as np
import time
import serial
from skimage import measure
import math

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi('GUI2.ui', self)
        self.show()

        self.cap = cv2.VideoCapture(1)
        self.scale_factor = 1
        

        self.portnum='COM16'
        self.connect_state = False
        self.Comport.setText(str(self.portnum))
        


        self.U_btn.clicked.connect(self.U_btnClick)
        self.D_btn.clicked.connect(self.D_btnClick)
        self.R_btn.clicked.connect(self.R_btnClick)
        self.L_btn.clicked.connect(self.L_btnClick)
        self.ATK_start_btn.clicked.connect(self.ATK_start_btnClick)
        self.DFD_start_btn.clicked.connect(self.DFD_start_btnClick)
        self.restart_btn.clicked.connect(self.restart_btnClick)
        self.Connect_btn.clicked.connect(self.Connect_btnClick)
        self.original_img.clicked.connect(self.Original_img_Click)
        self.mask_img.clicked.connect(self.Mask_img_Click)
        self.red_HSV_img.clicked.connect(self.Red_HSV_img_Click)
        self.green_HSV_img.clicked.connect(self.Green_HSV_img_Click)
        self.cross_mask_img.clicked.connect(self.Cross_Mask_img_Click)
        self.show_img_value = 1


        self.R_L1.valueChanged.connect(self.R_L1_slider_changed)
        self.R_L2.valueChanged.connect(self.R_L2_slider_changed)
        self.R_L3.valueChanged.connect(self.R_L3_slider_changed)
        self.R_H1.valueChanged.connect(self.R_H1_slider_changed)
        self.R_H1.valueChanged.connect(self.R_H1_slider_changed)
        self.R_H1.valueChanged.connect(self.R_H1_slider_changed)
        self.Cross_Thresh.valueChanged.connect(self.Cross_Thresh_changed)

        self.R_l1 = 0
        self.R_l2 = 34
        self.R_l3 = 180

        self.R_h1 = 66
        self.R_h2 = 255
        self.R_h3 = 255

        self.R_L1.setValue(self.R_l1)
        self.R_L2.setValue(self.R_l2)
        self.R_L3.setValue(self.R_l3)
        self.R_H1.setValue(self.R_h1)
        self.R_H2.setValue(self.R_h2)
        self.R_H3.setValue(self.R_h3)

        self.R_L1_text.setText(str(self.R_h1))
        self.R_L2_text.setText(str(self.R_l2))
        self.R_L3_text.setText(str(self.R_l3))
        self.R_H1_text.setText(str(self.R_h1))
        self.R_H2_text.setText(str(self.R_h2))
        self.R_H3_text.setText(str(self.R_h3))


        self.G_L1.valueChanged.connect(self.G_L1_slider_changed)
        self.G_L2.valueChanged.connect(self.G_L2_slider_changed)
        self.G_L3.valueChanged.connect(self.G_L3_slider_changed)
        self.G_H1.valueChanged.connect(self.G_H1_slider_changed)
        self.G_H1.valueChanged.connect(self.G_H1_slider_changed)
        self.G_H1.valueChanged.connect(self.G_H1_slider_changed)
        

        self.G_l1 = 71
        self.G_l2 = 101
        self.G_l3 = 206

        self.G_h1 = 132
    
        self.G_h2 = 255
        self.G_h3 = 255

        self.G_L1.setValue(self.G_l1)
        self.G_L2.setValue(self.G_l2)
        self.G_L3.setValue(self.G_l3)
        self.G_H1.setValue(self.G_h1)
        self.G_H2.setValue(self.G_h2)
        self.G_H3.setValue(self.G_h3)

        self.G_L1_text.setText(str(self.G_h1))
        self.G_L2_text.setText(str(self.G_l2))
        self.G_L3_text.setText(str(self.G_l3))
        self.G_H1_text.setText(str(self.G_h1))
        self.G_H2_text.setText(str(self.G_h2))
        self.G_H3_text.setText(str(self.G_h3))



        self.frame_count = 0
        self.start_time = time.time()

        self.timer = QtCore.QTimer(self)
        #self.timer2 = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        #self.timer2.timeout.connect(self.get_fps)
        if not self.timer.isActive():
            self.timer.start(33)
        #self.timer2.start(1000)
        #self.timer.start(30)
        self.goal_mean = []

        self.px_b = 0
        self.py_b = 0

        self.lazer_red_x = 0
        self.lazer_red_y = 0


        self.pan_kd = 0.001 # 작을수록 섬세해짐
        self.pan_kp = 0.01
        
        self.tilt_kd = 0.0015 # 작을수록 섬세해짐
        self.tilt_kp = 0.0125
        


        self.cross_thresh = 65
        self.gray_value = 65
        self.Cross_Thresh_text.setText(str(self.cross_thresh))
        self.pan_kp_value.setText(str(self.pan_kp))
        self.pan_kd_value.setText(str(self.pan_kd))


        self.tilt_kp_value.setText(str(self.tilt_kp))
        self.tilt_kd_value.setText(str(self.tilt_kd))

        self.prev_error = None

        self.ATK_start_flag = False
        self.DFD_start_flag = False
        #self.FPS_timer.start(1000 // self.cap.get(cv2.CAP_PROP_FPS))
        self.prev_time = 0

        self.ATK_Timer = QtCore.QTimer()
        self.ATK_Timer.setInterval(100)
        self.ATK_Timer.timeout.connect(self.ATK_time)
        self.is_running = False
        self.time =0


    def control(self, error):
        if self.prev_error is None:
            self.prev_error = error
        derivative = error - self.prev_error
        control = self.kp * error + self.kd * derivative
        self.prev_error = error
        return control


        
    def get_fps(self):
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.Frame.display(fps)
        #print(fps)
    

    def update_frame(self):
        ret, frame = self.cap.read()
        if ret:
            
            frame_height, frame_width, _ = frame.shape
    
            # 확대할 영역 계산
            x_start = int(frame_width / 2 - (frame_width / (2 * self.scale_factor)))
            y_start = int(frame_height / 2 - (frame_height / (2 * self.scale_factor)))
            x_end = int(frame_width / 2 + (frame_width / (2 * self.scale_factor)))
            y_end = int(frame_height / 2 + (frame_height / (2 * self.scale_factor)))
            
            # 영상 확대
            #zoomed_frame = frame[y_start:y_end, x_start:x_end]
            #cv2.imshow("",frame)
            #frame = cv2.resize(zoomed_frame, (frame_width, frame_height))
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            video = (
                ((frame[:, :, 2] >= (np.max(frame[:, :, 2])-2) )*255).astype(np.uint8)
            )
           
            k = cv2.getStructuringElement(cv2.MORPH_RECT, (8,8))
            # 모폴로지 연산 적용

            Mask = cv2.dilate(video, k)
            Mask2 = cv2.dilate(Mask, k)

            labeled_image = measure.label(Mask)

            # Get properties of each labeled region
            regions = measure.regionprops(labeled_image)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            


            # 빨간색 범위 정의
            
            lower_red = np.array([self.R_l1, self.R_l2, self.R_l3])
            upper_red = np.array([self.R_h1, self.R_h2, self.R_h3])

            lower_green = np.array([self.G_l1, self.G_l2, self.G_l3])
            upper_green = np.array([self.G_h1, self.G_h2, self.G_h3])



            # 빨간색 마스크 생성
            mask_red = cv2.inRange(hsv, lower_red, upper_red)
            #cv2.imshow("_",mask_red)
            # 녹색 마스크 생성
            mask_green = cv2.inRange(hsv, lower_green, upper_green)
            #cv2.imshow("__",mask_green)

            # 마스크 적용
            k = cv2.getStructuringElement(cv2.MORPH_RECT, (2,2))
            k2 = cv2.getStructuringElement(cv2.MORPH_RECT, (25,25))


            result_red = cv2.bitwise_and(Mask2,mask_red)
            result_red = cv2.erode(result_red, k)
            result_red = cv2.dilate(result_red, k2)
            

            result_green = cv2.bitwise_and(Mask2,mask_green)

            result_green = cv2.erode(result_green, k)
            result_green = cv2.dilate(result_green, k2)
            
            # 결과 이미지 출력
            contours_red, _ = cv2.findContours(result_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_green, _ = cv2.findContours(result_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours_red:
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    gap = 3
                    for region in regions:
                        if (region.bbox[0]-gap <=cy and cy <=region.bbox[2]+gap) and (
                            region.bbox[1]-gap <=cx and cx <=region.bbox[3]+gap):
                            #print(region.centroid)
                    
                    # 중심 그리기
                    
                            cv2.circle(frame, (int(region.centroid[1]), int(region.centroid[0])), 5, (0, 0, 255), -1)
                            self.lazer_x_red.setText("lazer_x (red) : " + str(int(region.centroid[1])))
                            self.lazer_y_red.setText("lazer_y (red) : " + str(int(region.centroid[0])))
                            self.lazer_red_x = region.centroid[1]
                            self.lazer_red_y = region.centroid[0]


            for contour in contours_green:
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    gap = 3
                    for region in regions:
                        if (region.bbox[0]-gap <=cy and cy <=region.bbox[2]+gap) and (
                            region.bbox[1]-gap <=cx and cx <=region.bbox[3]+gap):
                            #print(region.centroid)
                    
                    # 중심 그리기
                    
                            cv2.circle(frame, (int(region.centroid[1]), int(region.centroid[0])), 5, (0, 255, 0), -1)
                            self.lazer_x_green.setText("lazer_x (green) : " + str(int(region.centroid[1])))
                            self.lazer_y_green.setText("lazer_y (green) : " + str(int(region.centroid[0])))
                            self.lazer_green_x = region.centroid[1]
                            self.lazer_green_y = region.centroid[0]
                  
            kernel1 = np.ones((3, 3), np.uint8)
            frame_gray_tsh = np.where(frame_gray < self.gray_value, frame_gray, 255)
            frame_gray_tsh= cv2.dilate(frame_gray_tsh, kernel1)
            frame_gray_tsh= cv2.erode(frame_gray_tsh, kernel1)
            
            if self.frame_count %30 == 0:
                # 골대 인식 및 화면에 출력 (px_b, py_b)
                # 잘 잡히지 않을 시 goodFeaturesToTrack 파라미터 바꿔볼 것
                



                corners = cv2.goodFeaturesToTrack(
                    frame_gray_tsh, 15, 0.04, 1)

                # 골대가 비디오 내부에 잡힐 경우 좌표 (px_b,py_b)을 출력
                if corners is not None:
                    corners = np.int0(corners).reshape(
                        corners.shape[0], corners.shape[2])

                    xy_b_12 = corners[corners[:, 0].argsort(kind='mergesort')]
                    xy_b_34 = corners[corners[:, 1].argsort(kind='mergesort')]
                    x1_b, y1_b = xy_b_12[0]
                    x2_b, y2_b = xy_b_12[-1]
                    x3_b, y3_b = xy_b_34[0]
                    x4_b, y4_b = xy_b_34[-1]

                    self.px_b = ((x1_b * y2_b - y1_b * x2_b) * (x3_b - x4_b) - (x1_b - x2_b) * (x3_b * y4_b - y3_b * x4_b)) / \
                        ((x1_b - x2_b) * (y3_b - y4_b) - (y1_b - y2_b) * (x3_b - x4_b))

                    self.py_b = ((x1_b * y2_b - y1_b * x2_b) * (y3_b - y4_b) - (y1_b - y2_b) * (x3_b * y4_b - y3_b * x4_b)) / \
                        ((x1_b - x2_b) * (y3_b - y4_b) - (y1_b - y2_b) * (x3_b - x4_b))

            # 찾은 코너로 골대 좌표 찾기
                    if np.isnan(self.px_b) == False and np.isnan(self.py_b) == False:
                        # 골대가 비디오에 잡힐 경우 좌표 출력
                        cv2.circle(frame, (int(self.px_b), int(self.py_b)), 20, (255, 0, 0), 3)
                        self.goal_x.setText("goal_x : " + str(int(self.px_b)))
                        self.goal_y.setText("goal_y : " + str(int(self.py_b)))
                        self.goal_xy_x = self.px_b
                        self.goal_xy_y = self.py_b
                    for i in corners:
                        x_b, y_b = i.ravel()
                        cv2.circle(frame, (x_b, y_b), 3, (255, 0, 0), -1)
                else:
                    # 골대가 영상에 잡히지 않을 때 좌표 (0,0) 지정
                    self.goal_x.setText("goal_x : 0")
                    self.goal_y.setText("goal_y : 0")
            else:
                if not(np.isnan(self.px_b)):
                    cv2.circle(frame, (int(self.px_b), int(self.py_b)), 20, (255, 0, 0), 3)
            

            if self.connect_state:
                if self.ATK_start_flag:
                    if self.lazer_red_x is not None and self.goal_xy_x is not None:
                        self.move(self.lazer_red_x,self.lazer_red_y,self.goal_xy_x,self.goal_xy_y)

                elif self.DFD_start_flag:
                    if self.lazer_red_x is not None and self.goal_xy_x is not None:
                        rx,ry = self.find_Mid_point(self.lazer_red_x,self.lazer_red_y,self.goal_xy_x,self.goal_xy_y)
                        self.move(self.lazer_green_x,self.lazer_green_y,rx,ry)
                

                
            #비디오 FPS 출력
            self.frame_count += 1
            # elapsed_time = time.time() - self.start_time
            # fps = self.frame_count / elapsed_time
            #fps = 1.0 / self.timer.interval()


            current_time = cv2.getTickCount()
            fps = cv2.getTickFrequency() / (current_time - self.prev_time)
            self.prev_time = current_time
            if self.frame_count == 5:

                self.Frame.display(fps)
                self.frame_count =0

            # fps = 1.0 / self.timer.interval()
            #print("FPS:", fps)
            
            if (self.show_img_value==1):
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            elif self.show_img_value== 2:
                frame = cv2.cvtColor(Mask2, cv2.COLOR_BGR2RGB)
            elif self.show_img_value== 3:
                frame = cv2.cvtColor(mask_red, cv2.COLOR_BGR2RGB)
            elif self.show_img_value== 4:
                frame = cv2.cvtColor(mask_green, cv2.COLOR_BGR2RGB)
            elif self.show_img_value== 5:
                frame = cv2.cvtColor(frame_gray_tsh, cv2.COLOR_BGR2RGB)



            img = QtGui.QImage(frame, frame.shape[1], frame.shape[0], QtGui.QImage.Format_RGB888)
            pix = QtGui.QPixmap.fromImage(img)
            self.video_label.setPixmap(pix)



    def move(self,x,y,x1,y1): #십자 공격자
       
        error = self.distance(x,y,x1,y1)#(self.lazer_red_x,self.lazer_red_y,self.goal_xy_x,self.goal_xy_y)
        #print(error)
        if (error<15): 
            # if self.ATK_start_flag:
            #     self.ATK_start_flag = False
            self.ATK_Timer.stop()
            self.is_running = False
            return 0
        control = self.control(error)
        dx = control[0] * (x1 - x) / error
        dy = control[1] * (y1 - y) / error
        self.send_Serial("{:.3f}".format(dx),"{:.3f}".format(-dy))
        #time.sleep(0.3)
        #self.canvas.move(self.id, dx, dy)
        #self.pos[0] += dx
        #self.pos[1] += dy

    def distance(self,x,y,x1,y1):
        a=((x - x1) ** 2 + (y - y1) ** 2) ** 0.5
        #print(a)
        return a
    def find_Mid_point(self,red_x,red_y,goal_x,goal_y):
        distance = math.sqrt((red_x - goal_x)**2 + (red_y - goal_y)**2)
        x = goal_x + (red_x - goal_x) * 2.0 / 5.0
        y = goal_y + (red_y - goal_y) * 2.0 / 5.0

        return (x, y)

    def control(self, error):
        self.pan_kp = float(self.pan_kp_value.text())
        self.pan_kd = float(self.pan_kd_value.text())

        self.tilt_kp = float(self.tilt_kp_value.text())
        self.tilt_kd = float(self.tilt_kd_value.text())

        if self.prev_error is None:
            self.prev_error = error

        derivative = error - self.prev_error
        pan_control = self.pan_kp * error + self.pan_kd * derivative
        tilt_control = self.tilt_kp * error + self.tilt_kd * derivative
        
        self.prev_error = error
        return pan_control,tilt_control
    

    def send_Serial(self,x,y):
        commend='p'+str(x).zfill(4)+'t'+str(y).zfill(4)+"\n"
        self.ser.write(commend.encode())
       


    def U_btnClick(self):
        self.send_Serial(0,5)
    def D_btnClick(self):
        self.send_Serial(0,-5)
    def R_btnClick(self):
        self.send_Serial(5,0)
    def L_btnClick(self):
        self.send_Serial(-5,0)
    def ATK_start_btnClick(self):
        self.ATK_start_flag = True
        self.DFD_start_flag = False
        if not self.is_running:
            self.time = 0
            self.ATK_Timer.start()
            self.is_running = True

    def ATK_time(self):
        self.time+=0.1
        self.Play_Time.display(self.time)

    def DFD_start_btnClick(self):
        self.DFD_start_flag = True
        self.ATK_start_flag = False
    def restart_btnClick(self):
        if not(self.connect_state): return 0 
        if self.is_running:
            self.ATK_Timer.stop()
            self.is_running = False
        
        self.DFD_start_flag = False
        self.ATK_start_flag = False
        self.ser.close()
        self.ser=serial.Serial(port=str(self.Comport.text()),baudrate=115200)

    def Connect_btnClick(self):
        
        try:
            self.ser=serial.Serial(port=str(self.Comport.text()),baudrate=115200)
        except:
            self.Comport_state.setText("Error")
            return 0
        self.Comport_state.setText("Connect")
        self.connect_state = True




    def Original_img_Click(self):
        self.show_img_value = 1
    def Mask_img_Click(self):
        self.show_img_value = 2
    def Red_HSV_img_Click(self):
        self.show_img_value = 3
    def Green_HSV_img_Click(self):
        self.show_img_value = 4
    def Cross_Mask_img_Click(self):
        self.show_img_value = 5
        
    def G_L1_slider_changed(self, value):
        self.G_l1 = value
        self.G_L1_text.setText(str(value))
    def G_L2_slider_changed(self, value):
        self.G_l2 = value
        self.G_L2_text.setText(str(value))
    def G_L3_slider_changed(self, value):
        self.G_l3 = value
        self.G_L3_text.setText(str(value))
    def G_H1_slider_changed(self, value):
        self.G_h1 = value
        self.G_H1_text.setText(str(value))
    def G_H2_slider_changed(self, value):
        self.G_h2 = value
        self.G_H2_text.setText(str(value))
    def G_H3_slider_changed(self, value):
        self.G_h3 = value
        self.G_H3_text.setText(str(value))    
    def R_L1_slider_changed(self, value):
        self.R_l1 = value
        self.R_L1_text.setText(str(value))
    def R_L2_slider_changed(self, value):
        self.R_l2 = value
        self.R_L2_text.setText(str(value))
    def R_L3_slider_changed(self, value):
        self.R_l3 = value
        self.R_L3_text.setText(str(value))
    def R_H1_slider_changed(self, value):
        self.R_h1 = value
        self.R_H1_text.setText(str(value))
    def R_H2_slider_changed(self, value):
        self.R_h2 = value
        self.R_H2_text.setText(str(value))
    def R_H3_slider_changed(self, value):
        self.R_h3 = value
        self.R_H3_text.setText(str(value))    
    def Cross_Thresh_changed(self, value):
        self.gray_value = value
        self.Cross_Thresh_text.setText(str(value)) 

app = QtWidgets.QApplication(sys.argv)
window = MainWindow()
app.exec_()