#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
from rospkg import RosPack
rp = RosPack()
sys.path.append(rp.get_path('canopen_communication') + "/modular/")
# sys.path.append(rp.get_path('canopen_communication') + "/file/")

from modular_G100 import G100
from modular_I100 import I100
from modular_T100 import T100
from  math import fabs,degrees,radians
from PyQt5.QtWidgets import QWidget,QDesktopWidget,QMessageBox
from PyQt5.QtCore import QThread,pyqtSignal
import time
import traceback
import rospy
from std_msgs.msg import Float64MultiArray

class Thread_transmit_joint_data(QThread):

    # 发送通信是否成功
    sin_init_success = pyqtSignal()
    sin_init_error = pyqtSignal()
    # 各窗口反馈数据信号
    sin_joint_control_actual_joint_data = pyqtSignal(list)
    sin_descartes_control_actual_joint_data = pyqtSignal(list)
    sin_offline_data_actual_joint_data = pyqtSignal(list)

    def __init__(self,parent=None):
        
        super(Thread_transmit_joint_data, self).__init__(parent)
        self.eds_file = rp.get_path('canopen_communication') + "/file/Copley.eds"
        #print path.exists(eds_file)

        # 用于接收逆解
        # this publisher was to sent desacrtes points by inverse algorithm to getting the joint datas.
        self.pub_wdc = rospy.Publisher('climbot5d_Descartes_point', Float64MultiArray , queue_size = 5)
        # this subscriber was to receive the joint data.
        self.sub_wdc = rospy.Subscriber('climbot5d_Inverse_solution', Float64MultiArray , self.Inverse_solution_callback)
        self.descartes_point = Float64MultiArray()
        self.inverse_solution = Float64MultiArray()
        self.descartes_point.data = [1]*18
        self.inverse_solution.data = [1]*10
        
        # 关节数据
        self.__G0_data = 0.0
        self.__I1_data = 0.0
        self.__T2_data = 0.0
        self.__T3_data = 0.0
        self.__T4_data = 0.0
        self.__I5_data = 0.0
        self.__G6_data = 0.0
        # 设置零点使用
        self.__I1_error = 0
        self.__T2_error = 0
        self.__T3_error = 0
        self.__T4_error = 0
        self.__I5_error = 0
        # 用于判断关节是否为新值
        self.__G0_old_data = 0.0
        self.__I1_old_data = 0.0
        self.__T2_old_data = 0.0
        self.__T3_old_data = 0.0
        self.__T4_old_data = 0.0
        self.__I5_old_data = 0.0
        self.__G6_old_data = 0.0
        # 关节速度
        self.__I1_velocity = 0
        self.__T2_velocity = 0
        self.__T3_velocity = 0
        self.__T4_velocity = 0
        self.__I5_velocity = 0
        # 用于判断关节速度是否为新值
        self.__I1_old_velocity = 0
        self.__T2_old_velocity = 0 
        self.__T3_old_velocity = 0
        self.__T4_old_velocity = 0
        self.__I5_old_velocity = 0
        # 末端笛卡尔点初始姿态
        self.__Descartes_X = 0.5864
        self.__Descartes_Y = 0
        self.__Descartes_Z = 0
        self.__Descartes_RX = 0
        self.__Descartes_RY = 0
        self.__Descartes_RZ = 3.141592654
        # 末端笛卡尔点初始速度
        self.__X_velocity = 0
        self.__Y_velocity = 0
        self.__Z_velocity = 0
        self.__RX_velocity = 0
        self.__RY_velocity = 0
        self.__RZ_velocity = 0 
        # 运行模式
        self.__position_mode = False
        self.__velocity_mode = False
        self.__gripper_mode = False
        # 停止方式
        self.stop = False
        self.quick_stop = False
        # 判断采用哪种反馈
        self.__feedback_joint = False
        self.__feedback_descartes = False
        self.__feedback_offline = False
        # 离线数据索引
        self.__index = 0
        # 存储末端笛卡尔点位置,速度
        self.__descartes_data = []
        # 存储离线数据
        self.__offline_data = []
        # 关节误差
        self.__error_joint = 4e-3

    def run(self):
        try:
            self.__start_communication()
            # print "communication success"
            pass
        except Exception as e:
            self.sin_init_error.emit()
            traceback.print_exc()
            # print e
        else:
            self.sin_init_success.emit()

            while not (self.quick_stop or self.stop):


                if self.__position_mode:  

                    if self.__T2_old_data != self.__T2_data:
                        if abs(self.__T2_data - self.__T2_error + 1.57079) <= 2.07:   # 1.57079 = 90deg  T2 real init position
                            self.T2.sent_position(self.__T2_data,self.__T2_velocity)
                            self.__T2_old_data = self.__T2_data

                    if self.__T3_old_data != self.__T3_data:
                        if abs(self.__T3_data - self.__T3_error) <= 2.07: 
                            self.T3.sent_position(self.__T3_data,self.__T3_velocity)   
                            self.__T3_old_data = self.__T3_data

                    if self.__T4_old_data != self.__T4_data:
                        if abs(self.__T4_data - self.__T4_error + 1.57079) <= 2.07:   # 1.57079 = 90deg  T2 real init position
                            self.T4.sent_position(self.__T4_data,self.__T4_velocity)         
                            self.__T4_old_data = self.__T4_data

                    if self.__I1_old_data != self.__I1_data:
                        if abs(self.__I1_data - self.__I1_error) <= 3.1416: 
                            self.I1.sent_position(self.__I1_data,self.__I1_velocity)
                            self.__I1_old_data = self.__I1_data

                    if self.__I5_old_data != self.__I5_data:
                        if abs(self.__I5_data - self.__I5_error) <= 3.1416:
                            self.I5.sent_position(self.__I5_data,self.__I5_velocity)  
                            self.__I5_old_data = self.__I5_data
                    self.__position_mode = False

                elif self.__velocity_mode:

                    if self.__T2_old_velocity != self.__T2_velocity:
                        self.T2.sent_velocity(self.__T2_velocity)
                        self.__T2_old_velocity = self.__T2_velocity

                    if self.__T3_old_velocity != self.__T3_velocity:
                        self.T3.sent_velocity(self.__T3_velocity)
                        self.__T3_old_velocity = self.__T3_velocity

                    if self.__T4_old_velocity != self.__T4_velocity:
                        self.T4.sent_velocity(self.__T4_velocity)
                        self.__T4_old_velocity = self.__T4_velocity

                    if self.__I1_old_velocity != self.__I1_velocity:
                        self.I1.sent_velocity(self.__I1_velocity)
                        self.__I1_old_velocity = self.__I1_velocity         

                    if self.__I5_old_velocity != self.__I5_velocity:
                        self.I5.sent_velocity(self.__I5_velocity)
                        self.__I5_old_velocity = self.__I5_velocity
                    self.__velocity_mode = False                                                    

                elif self.__gripper_mode:

                    if self.__G6_old_data != self.__G6_data:
                        self.G6.sent_torque(self.__G6_data)
                        self.__G6_old_data = self.__G6_data

                    elif self.__G0_old_data != self.__G0_data:
                        self.G0.sent_torque(self.__G0_data)
                        self.__G0_old_data = self.__G0_data
                    
                    self.__gripper_mode = False
                pass

                try:
                    self.__feedback = [self.G0.get_torque(),self.I1.get_position(),self.T2.get_position(),self.T3.get_position(),\
                                    self.T4.get_position(),self.I5.get_position(),self.G6.get_torque()]
                    # self.__feedback = [self.__G0_data,self.__I1_data,self.__T2_data,self.__T3_data,self.__T4_data,self.__I5_data,\
                                        # self.__G6_data]
                    if self.__feedback_joint:
                        self.sin_joint_control_actual_joint_data.emit(self.__feedback)
                        
                    elif self.__feedback_descartes:
                        self.sin_descartes_control_actual_joint_data.emit(self.__feedback)
                            
                    elif self.__feedback_offline:
                        self.sin_offline_data_actual_joint_data.emit(self.__feedback)
                        self.__choice_offline()
                    # time.sleep(0.01)

                except Exception as e:
                    traceback.print_exc()
                    # print e
            else:
                if self.quick_stop:
                    self.G0.quick_stop()
                    self.I1.quick_stop()
                    self.T2.quick_stop()
                    self.T3.quick_stop()
                    self.T4.quick_stop()
                    self.I5.quick_stop()
                    self.G6.quick_stop()
                    pass
                    
                else:
                    self.G0.stop()
                    self.I1.stop()
                    self.T2.stop()
                    self.T3.stop()
                    self.T4.stop()
                    self.I5.stop()
                    self.G6.stop()
                    pass            

    def if_stop(self):
        self.stop = True

    def if_quick_stop(self):
        self.quick_stop = True

    # 开始通信
    def __start_communication(self):

            self.I1 = I100(1,self.eds_file)
            self.T2 = T100(2,self.eds_file)
            self.T3 = T100(3,self.eds_file)
            self.T4 = T100(4,self.eds_file)
            self.I5 = I100(5,self.eds_file)
            self.G0 = G100(6,self.eds_file)
            self.G6 = G100(7,self.eds_file)

            self.G0.start()
            self.I1.start()
            self.T2.start()
            self.T3.start()
            self.T4.start()
            self.I5.start()
            self.G6.start()

            # set position mode.
            self.set__position_mode()

    # 逆解回调
    def Inverse_solution_callback(self, msg):
        self.inverse_solution = msg

    # 关节空间,速度模式发送关节数据
    def joint_sent_data(self,velocity):
        
        if velocity[0] == 1:
            self.__I1_velocity = velocity[1]
        elif velocity[0]  == 2:
            self.__T2_velocity = velocity[1]
        elif velocity[0]  == 3:
            self.__T3_velocity = velocity[1]
        elif velocity[0]  == 4:
            self.__T4_velocity = velocity[1]
        elif velocity[0]  == 5:
            self.__I5_velocity = velocity[1]

        self.__position_mode = False
        self.__velocity_mode = True

        self.__feedback_joint = True
        self.__feedback_descartes = False
        self.__feedback_offline = False          
        pass
    
    # 笛卡尔空间,位置模式发送关节位置
    def descartes_sent_data(self,data):

        self.__descartes_index, self.__descartes_data = data
        if self.__descartes_index == 1:
            self.__Descartes_X ,self.__X_velocity = self.__descartes_data
        elif self.__descartes_index == 2:
            self.__Descartes_Y,self.__Y_velocity = self.__descartes_data
        elif self.__descartes_index == 3:
            self.__Descartes_Z,self.__Z_velocity = self.__descartes_data
        elif self.__descartes_index == 4:
            self.__Descartes_RX,self.__RX_velocity = self.__descartes_data
        elif self.__descartes_index == 5:
            self.__Descartes_RY,self.__RY_velocity = self.__descartes_data
        elif self.__descartes_index == 6:
            self.__Descartes_RZ,self.__RZ_velocity = self.__descartes_data

        self.__get_inverse_solution()

        self.__position_mode = True
        self.__velocity_mode = False

        self.__feedback_joint = False
        self.__feedback_descartes = True
        self.__feedback_offline = False 

    # 速度模式下,发送离线数据     
    def offline_sent_data(self,data):

        self.__offline_data , self.__feedback_offline = data
        self.__index = 0

        self.__I1_data = self.__offline_data[self.__index][0]
        self.__T2_data = self.__offline_data[self.__index][1]
        self.__T3_data = self.__offline_data[self.__index][2]
        self.__T4_data = self.__offline_data[self.__index][3]
        self.__I5_data = self.__offline_data[self.__index][4]

        self.__I1_velocity = self.__offline_data[self.__index][5]
        self.__T2_velocity = self.__offline_data[self.__index][6]
        self.__T3_velocity = self.__offline_data[self.__index][7]
        self.__T4_velocity = self.__offline_data[self.__index][8]
        self.__I5_velocity = self.__offline_data[self.__index][9]

        self.__velocity_mode = True
        self.__position_mode = False

        self.__feedback_joint = False
        self.__feedback_descartes = False
        self.__feedback_offline = True 

    # 发送夹持器数据
    def new_G0_joint_data(self,data):
        self.__G0_data = data
        self.__gripper_mode = True

    def new_G6_joint_data(self,data):
        self.__G6_data = data
        self.__gripper_mode = True

    # 获取逆解
    def __get_inverse_solution(self):

        self.descartes_point.data[1] = self.__Descartes_X
        self.descartes_point.data[2] = self.__Descartes_Y
        self.descartes_point.data[3] = self.__Descartes_Z
        self.descartes_point.data[4] = self.__Descartes_RX
        self.descartes_point.data[5] = self.__Descartes_RY
        self.descartes_point.data[6] = self.__Descartes_RZ

        self.descartes_point.data[7] = self.__I1_data
        self.descartes_point.data[8] = - self.__T2_data
        self.descartes_point.data[9] = - self.__T3_data
        self.descartes_point.data[10] = - self.__T4_data    
        self.descartes_point.data[11] = self.__I5_data

        self.descartes_point.data[12] = self.__X_velocity
        self.descartes_point.data[13] = self.__Y_velocity
        self.descartes_point.data[14] = self.__Z_velocity
        self.descartes_point.data[15] = self.__RX_velocity
        self.descartes_point.data[16] = self.__RY_velocity
        self.descartes_point.data[17] = self.__RZ_velocity

        self.pub_wdc.publish(self.descartes_point)
        try:
            rospy.wait_for_message("climbot5d_Inverse_solution", Float64MultiArray,timeout=3)
        except:
            pass
        self.__I1_data = self.inverse_solution.data[0]
        self.__T2_data = self.inverse_solution.data[1]
        self.__T3_data = self.inverse_solution.data[2]
        self.__T4_data = self.inverse_solution.data[3]
        self.__I5_data = self.inverse_solution.data[4]
        self.__I1_velocity = self.inverse_solution.data[5]
        self.__T2_velocity = self.inverse_solution.data[6]
        self.__T3_velocity = self.inverse_solution.data[7]
        self.__T4_velocity = self.inverse_solution.data[8]
        self.__I5_velocity = self.inverse_solution.data[9]
        pass                      

    # 判断是否输入下一点离线数据
    def __choice_offline(self):

        if( abs(self.__offline_data[self.__index][0] - self.__feedback[1])<self.__error_joint and
            abs(self.__offline_data[self.__index][1] - self.__feedback[2])<self.__error_joint and
            abs(self.__offline_data[self.__index][2] - self.__feedback[3])<self.__error_joint and
            abs(self.__offline_data[self.__index][3] - self.__feedback[4])<self.__error_joint and
            abs(self.__offline_data[self.__index][4] - self.__feedback[5])<self.__error_joint):
            # print 1
            self.__index  += 1

            if self.__index < len(self.__offline_data):
                self.__I1_data = self.__offline_data[self.__index][0]
                self.__T2_data = self.__offline_data[self.__index][1]
                self.__T3_data = self.__offline_data[self.__index][2]
                self.__T4_data = self.__offline_data[self.__index][3]
                self.__I5_data = self.__offline_data[self.__index][4]
                
                if self.__I1_velocity != self.__offline_data[self.__index][5]:
                    self.__I1_velocity = self.__offline_data[self.__index][5]
                if self.__T2_velocity != self.__offline_data[self.__index][6]:
                    self.__T2_velocity = self.__offline_data[self.__index][6]
                if self.__T3_velocity != self.__offline_data[self.__index][7]:
                    self.__T3_velocity = self.__offline_data[self.__index][7]
                if self.__T4_velocity != self.__offline_data[self.__index][8]:
                    self.__T4_velocity = self.__offline_data[self.__index][8]
                if self.__I5_velocity != self.__offline_data[self.__index][9]:
                    self.__I5_velocity = self.__offline_data[self.__index][9]

                self.__velocity_mode = True
            else:
                self.__velocity_mode = False
                self.__feedback_offline = False
                self.I1.sent_velocity(0)
                self.T2.sent_velocity(0)
                self.T3.sent_velocity(0)   
                self.T4.sent_velocity(0)         
                self.I5.sent_velocity(0)
            pass

    # 暂停运行
    def pause_run(self,data):

        try:
            if data:
                self.I1.pause_run()
                self.T2.pause_run()
                self.T3.pause_run()
                self.T4.pause_run()
                self.I5.pause_run()
                pass
            else:
                self.I1.continue_run()
                self.T2.continue_run()
                self.T3.continue_run()
                self.T4.continue_run()
                self.I5.continue_run()

                if self.__feedback_offline:
                    self.I1.sent_velocity(self.__I1_velocity)
                    self.T2.sent_velocity(self.__T2_velocity)
                    self.T3.sent_velocity(self.__T3_velocity)   
                    self.T4.sent_velocity(self.__T4_velocity)         
                    self.I5.sent_velocity(self.__I5_velocity)
                    pass
        except Exception as e:
            traceback.print_exc()
            print e
    
    # 获取零点位置
    def get_zero_position(self,data):
        self.__I1_error,self.__T2_error,self.__T3_error,self.__T4_error,self.__I5_error = data

    # 获取当前末端笛卡尔点位姿
    def get_init_descartes_point(self,data):
        self.__Descartes_X ,self.__Descartes_Y,self.__Descartes_Z,self.__Descartes_RX,self.__Descartes_RY,self.__Descartes_RZ = data

    # 设置速度模式
    def set_velocity_mode(self):

        self.I1.set_mode(3)
        self.T2.set_mode(3)
        self.T3.set_mode(3)
        self.T4.set_mode(3)
        self.I5.set_mode(3)
        # print 'success'
        pass
    
    # 设置位置模式
    def set__position_mode(self):

        self.I1.set_mode(1)
        self.T2.set_mode(1)
        self.T3.set_mode(1)
        self.T4.set_mode(1)
        self.I5.set_mode(1)
        pass
    
    # 回零
    def return_zero(self):
        self.set__position_mode()
        self.__I1_data = self.__I1_error
        self.__T2_data = self.__T2_error
        self.__T3_data = self.__T3_error
        self.__T4_data = self.__T4_error
        self.__I5_data = self.__I5_error
        self.__position_mode = True
        pass   