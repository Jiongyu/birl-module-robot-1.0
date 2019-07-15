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

class Thread_transmit_joint_data_wheel(QThread):

    # 发送通信是否成功
    sin_init_success = pyqtSignal()
    sin_init_error = pyqtSignal()
    # 各窗口反馈数据信号
    sin_joint_control_actual_joint_data = pyqtSignal(list)
    sin_descartes_control_actual_joint_data = pyqtSignal(list)
    sin_offline_data_actual_joint_data = pyqtSignal(list)

    def __init__(self,parent=None):
        
        super(Thread_transmit_joint_data_wheel, self).__init__(parent)
        self.eds_file = rp.get_path('canopen_communication') + "/file/Copley.eds"
        #print path.exists(eds_file)

        
        # 关节数据
        self.__I1_data = 0.0
        self.__T2_data = 0.0
        self.__T3_data = 0.0
        self.__I4_data = 0.0
        self.__T5_data = 0.0
        self.__T6_data = 0.0
        self.__I7_data = 0.0
        
        # 设置零点使用
        self.__I1_error = 0
        self.__T2_error = 0
        self.__T3_error = 0
        self.__I4_error = 0
        self.__T5_error = 0
        self.__T6_error = 0
        self.__I7_error = 0

        # 用于判断关节是否为新值
        self.__I1_old_data = 0.0
        self.__T2_old_data = 0.0
        self.__T3_old_data = 0.0
        self.__I4_old_data = 0.0
        self.__T5_old_data = 0.0
        self.__T6_old_data = 0.0
        self.__I7_old_data = 0.0

        # 关节速度
        self.__I1_velocity = 0
        self.__T2_velocity = 0
        self.__T3_velocity = 0
        self.__I4_velocity = 0
        self.__T5_velocity = 0
        self.__T6_velocity = 0
        self.__I7_velocity = 0

        # 用于判断关节速度是否为新值
        self.__I1_old_velocity = 0
        self.__T2_old_velocity = 0 
        self.__T3_old_velocity = 0
        self.__I4_old_velocity = 0
        self.__T5_old_velocity = 0
        self.__T6_old_velocity = 0
        self.__I7_old_velocity = 0

        self.__I1_reached = False
        self.__T2_reached = False
        self.__T3_reached = False
        self.__I4_reached = False
        self.__T5_reached = False
        self.__T6_reached = False
        self.__I7_reached = False

        # 运行模式
        self.__position_mode = False
        self.__velocity_mode = False
        # 停止方式
        self.stop = False
        self.quick_stop = False
        # 判断采用哪种反馈
        self.__feedback_joint = False
        self.__feedback_offline = False
        # 离线数据索引
        self.__index = 0
        # 存储离线数据
        self.__offline_data = []
        # 关节误差
        self.__error_joint = 1e-2

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

                # start = time.clock()

                if self.__position_mode:  

                    if self.__T2_old_data != self.__T2_data:
                        if abs(self.__T2_data - self.__T2_error ) <= 2.07:   
                            self.__T2_old_data = self.__T2_data

                    if self.__T3_old_data != self.__T3_data:
                        if abs(self.__T3_data - self.__T3_error - 1.57079) <= 2.07: # 1.57079 = 90deg  T3 real init position
                            self.T2.sent_position(self.__T2_data,self.__T2_velocity)
                            self.T3.sent_position(self.__T3_data,self.__T3_velocity)   
                            self.__T3_old_data = self.__T3_data

                    if self.__I4_old_data != self.__I4_data:
                        if abs(self.__I4_data - self.__I4_error ) <= 2.07:   
                            self.I4.sent_position(self.__I4_data,self.__I4_velocity)         
                            self.__I4_old_data = self.__I4_data

                    if self.__I1_old_data != self.__I1_data:
                            self.I1.sent_position(self.__I1_data,self.__I1_velocity)
                            self.__I1_old_data = self.__I1_data

                    if self.__T5_old_data != self.__T5_data:
                        if abs(self.__T5_data - self.__T5_error - 1.57079) <= 3.1416: # 1.57079 = 90deg  T5 real init position
                            self.T5.sent_position(self.__T5_data,self.__T5_velocity)  
                            self.__T5_old_data = self.__T5_data

                    if self.__T6_old_data != self.__T6_data:
                        if abs(self.__T6_data - self.__T6_error) <= 2.07:
                            self.T6.sent_position(self.__T6_data,self.__T6_velocity)  
                            self.__T6_old_data = self.__T6_data

                    if self.__I7_old_data != self.__I7_data:
                            self.I7.sent_position(self.__I7_data,self.__I7_velocity)  
                            self.__I7_old_data = self.__I7_data
                    self.__position_mode = False

                elif self.__velocity_mode:

                    if self.__T2_old_velocity != self.__T2_velocity:
                        self.T2.sent_velocity(self.__T2_velocity)
                        self.__T2_old_velocity = self.__T2_velocity

                    if self.__T3_old_velocity != self.__T3_velocity:
                        self.T3.sent_velocity(self.__T3_velocity)
                        self.__T3_old_velocity = self.__T3_velocity

                    if self.__I4_old_velocity != self.__I4_velocity:
                        self.I4.sent_velocity(self.__I4_velocity)
                        self.__I4_old_velocity = self.__I4_velocity

                    if self.__I1_old_velocity != self.__I1_velocity:
                        self.I1.sent_velocity(self.__I1_velocity)
                        self.__I1_old_velocity = self.__I1_velocity 

                    if self.__T5_old_velocity != self.__T5_velocity:
                        self.T5.sent_velocity(self.__T5_velocity)
                        self.__T5_old_velocity = self.__T5_velocity                        

                    if self.__T6_old_velocity != self.__T6_velocity:
                        self.T6.sent_velocity(self.__T6_velocity)
                        self.__T6_old_velocity = self.__T6_velocity

                    if self.__I7_old_velocity != self.__I7_velocity:
                        self.I7.sent_velocity(self.__I7_velocity)
                        self.__I7_old_velocity = self.__I7_velocity

                    self.__velocity_mode = False                                                    
                pass

                try:
                    self.__feedback = [self.I1.get_position(),self.T2.get_position(),self.T3.get_position(),\
                                    self.I4.get_position(),self.T5.get_position(),self.T6.get_position(),self.I7.get_position()]

                    if self.__feedback_joint:
                        self.sin_joint_control_actual_joint_data.emit(self.__feedback)
                            
                    elif self.__feedback_offline:
                        self.sin_offline_data_actual_joint_data.emit(self.__feedback)
                        self.__choice_offline()
                    time.sleep(0.001)

                except Exception as e:
                    traceback.print_exc()
                
                # end = time.clock()
                # print (end -start)
                # print "\n"

            else:
                if self.quick_stop:
                    self.I1.quick_stop()
                    self.T2.quick_stop()
                    self.T3.quick_stop()
                    self.I4.quick_stop()
                    self.T5.quick_stop()
                    self.T6.quick_stop()
                    self.I7.quick_stop()                
                    pass
                    
                else:
                    self.I1.stop()
                    self.T2.stop()
                    self.T3.stop()
                    self.I4.stop()
                    self.T5.stop()
                    self.T6.stop()
                    self.I7.stop()

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
            self.I4 = I100(4,self.eds_file)
            self.T5 = T100(5,self.eds_file)
            self.T6 = T100(6,self.eds_file)
            self.I7 = I100(7,self.eds_file)

            self.I1.start()
            self.T2.start()
            self.T3.start()
            self.I4.start()
            self.T5.start()
            self.T6.start()
            self.I7.start()
            # set position mode.
            self.set_velocity_mode()

    # 关节空间,速度模式发送关节数据
    def joint_sent_data(self,velocity):
        
        if velocity[0] == 1:
            self.__I1_velocity = velocity[1]
        elif velocity[0]  == 2:
            self.__T2_velocity = velocity[1]
        elif velocity[0]  == 3:
            self.__T3_velocity = velocity[1]
        elif velocity[0]  == 4:
            self.__I4_velocity = velocity[1]
        elif velocity[0]  == 5:
            self.__T5_velocity = velocity[1]
        elif velocity[0]  == 6:
            self.__T6_velocity = velocity[1]
        elif velocity[0]  == 7:
            self.__I7_velocity = velocity[1]

        self.__position_mode = False
        self.__velocity_mode = True

        self.__feedback_joint = True
        self.__feedback_offline = False          
        pass

    # 速度模式下,发送离线数据     
    def offline_sent_data(self,data):

        # print data

        self.__offline_data , self.__feedback_offline , self.__joint_velocity = data
        self.__index = 0

        # print self.__offline_data
        # print self.__joint_velocity

        self.__I1_data = self.__offline_data[self.__index][0]
        self.__T2_data = self.__offline_data[self.__index][1]
        self.__T3_data = self.__offline_data[self.__index][2]
        self.__I4_data = self.__offline_data[self.__index][3]
        self.__T5_data = self.__offline_data[self.__index][4]
        self.__T6_data = self.__offline_data[self.__index][5]
        self.__I7_data = self.__offline_data[self.__index][6]

        self.__I1_velocity = self.__joint_velocity[self.__index][0] 
        self.__T2_velocity = self.__joint_velocity[self.__index][1]
        self.__T3_velocity = self.__joint_velocity[self.__index][2]
        self.__I4_velocity = self.__joint_velocity[self.__index][3]
        self.__T5_velocity = self.__joint_velocity[self.__index][4]
        self.__T6_velocity = self.__joint_velocity[self.__index][5]
        self.__I7_velocity = self.__joint_velocity[self.__index][6]

        self.__velocity_mode = True
        self.__position_mode = False

        self.__feedback_joint = False
        self.__feedback_offline = True 

    # 判断是否输入下一点离线数据
    def __choice_offline(self):

        # print self.__T2_data
        # print self.__feedback[1]    
        # print self.__T2_data - self.__feedback[1]
        # print '\n'

        if( self.__I1_reached  and
            self.__T2_reached  and
            self.__T3_reached  and
            self.__I4_reached  and
            self.__T5_reached  and
            self.__T6_reached  and
            self.__I7_reached 
            ):

            self.__index += 1
            print self.__index

            if self.__index < len(self.__offline_data):

                print self.__offline_data[self.__index]

                self.__I1_data = self.__offline_data[self.__index][0]
                self.__T2_data = self.__offline_data[self.__index][1]
                self.__T3_data = self.__offline_data[self.__index][2]
                self.__I4_data = self.__offline_data[self.__index][3]
                self.__T5_data = self.__offline_data[self.__index][4]
                self.__T6_data = self.__offline_data[self.__index][5]
                self.__I7_data = self.__offline_data[self.__index][6]

                self.__I1_velocity = self.__joint_velocity[self.__index][0]
                self.__T2_velocity = self.__joint_velocity[self.__index][1]
                self.__T3_velocity = self.__joint_velocity[self.__index][2]
                self.__I4_velocity = self.__joint_velocity[self.__index][3]
                self.__T5_velocity = self.__joint_velocity[self.__index][4]
                self.__T6_velocity = self.__joint_velocity[self.__index][5]
                self.__I7_velocity = self.__joint_velocity[self.__index][6]

                self.__I1_reached = False 
                self.__T2_reached = False 
                self.__T3_reached = False 
                self.__I4_reached = False 
                self.__T5_reached = False 
                self.__T6_reached = False 
                self.__I7_reached = False

                self.__velocity_mode = True 

            else:
                print "end"

                self.__velocity_mode = False
                self.__feedback_offline = False
                self.I1.sent_velocity(0)
                self.T2.sent_velocity(0)
                self.T3.sent_velocity(0)   
                self.I4.sent_velocity(0)         
                self.T5.sent_velocity(0)
                self.T6.sent_velocity(0)
                self.I7.sent_velocity(0)            
            pass



        # print abs(self.__I1_data - self.__feedback[0] - self.__I1_error)
        # print abs(self.__T2_data - self.__feedback[1] - self.__T2_error)
        # print abs(self.__T3_data - self.__feedback[2] - self.__T3_error)
        # print abs(self.__I4_data - self.__feedback[3] - self.__I4_error)
        # print abs(self.__T5_data - self.__feedback[4] - self.__T5_error)
        # print abs(self.__T6_data - self.__feedback[5] - self.__T6_error)
        # print abs(self.__I7_data - self.__feedback[6] - self.__I7_error)
        # print "\n"
        # print self.__I1_reached
        # print self.__T2_reached
        # print self.__T3_reached
        # print self.__I4_reached
        # print self.__T5_reached
        # print self.__T6_reached
        # print self.__I7_reached
        # print "\n"

        if( abs(self.__I1_data - self.__feedback[0] - self.__I1_error) <= self.__error_joint):
            self.__I1_reached = True
            self.__I1_velocity = 0
            self.__velocity_mode = True

        if(abs(self.__T2_data - self.__feedback[1] - self.__T2_error) <= self.__error_joint):
            self.__T2_reached = True
            self.__T2_velocity = 0
            self.__velocity_mode = True

        if(abs(self.__T3_data - self.__feedback[2] - self.__T3_error) <= self.__error_joint):
            self.__T3_velocity = 0
            self.__T3_reached = True
            self.__velocity_mode = True

        if(abs(self.__I4_data - self.__feedback[3] - self.__I4_error) <= self.__error_joint):
            self.__I4_velocity = 0
            self.__I4_reached = True
            self.__velocity_mode = True

        if(abs(self.__T5_data - self.__feedback[4] - self.__T5_error) <= self.__error_joint):
            self.__T5_velocity = 0
            self.__T5_reached = True
            self.__velocity_mode = True

        if(abs(self.__T6_data - self.__feedback[5] - self.__T6_error) <= self.__error_joint):
            self.__T6_velocity = 0
            self.__T6_reached = True
            self.__velocity_mode = True

        if(abs(self.__I7_data - self.__feedback[6] - self.__I7_error) <= self.__error_joint):
            self.__I7_velocity = 0 
            self.__I7_reached = True
            self.__velocity_mode = True
         
        

    # 暂停运行
    def pause_run(self,data):

        try:
            if data:
                # self.I1.pause_run()
                # self.T2.pause_run()
                # self.T3.pause_run()
                # self.I4.pause_run()
                # self.T5.pause_run()
                # self.T6.pause_run()
                # self.I7.pause_run()
                self.I1.sent_velocity(0)
                self.T2.sent_velocity(0)
                self.T3.sent_velocity(0)   
                self.I4.sent_velocity(0)         
                self.T5.sent_velocity(0)
                self.T6.sent_velocity(0)
                self.I7.sent_velocity(0) 
                pass
            else:
                # self.I1.continue_run()
                # self.T2.continue_run()
                # self.T3.continue_run()
                # self.I4.continue_run()
                # self.T5.continue_run()
                # self.T6.continue_run()
                # self.I7.continue_run()
                self.__choice_offline()

                if self.__feedback_offline:
                    if (not self.__I1_reached):
                        self.I1.sent_velocity(self.__I1_velocity)
                    if (not self.__T2_reached):
                        self.T2.sent_velocity(self.__T2_velocity)
                    if (not self.__T3_velocity):
                        self.T3.sent_velocity(self.__T3_velocity)   
                    if (not self.__I4_reached):
                        self.I4.sent_velocity(self.__I4_velocity)         
                    if (not self.__T5_reached):
                        self.T5.sent_velocity(self.__T5_velocity)
                    if (not self.__T6_reached):
                        self.T6.sent_velocity(self.__T6_velocity)
                    if (not self.__I7_reached):
                        self.I7.sent_velocity(self.__I7_velocity)
                    pass
        except Exception as e:
            traceback.print_exc()
            print e
    
    # 获取零点位置
    def get_zero_position(self,data):
        self.__I1_error,self.__T2_error,self.__T3_error,self.__I4_error,self.__T5_error,self.__T6_error,self.__I7_error = data


    # 设置速度模式
    def set_velocity_mode(self):

        self.I1.set_mode(3)
        self.T2.set_mode(3)
        self.T3.set_mode(3)
        self.I4.set_mode(3)
        self.T5.set_mode(3)
        self.T6.set_mode(3)
        self.I7.set_mode(3)      
        # print 'success'
        pass
    
    # 设置位置模式
    def set__position_mode(self):

        self.I1.set_mode(1)
        self.T2.set_mode(1)
        self.T3.set_mode(1)
        self.I4.set_mode(1)
        self.T5.set_mode(1)
        self.T6.set_mode(1)
        self.I7.set_mode(1)
        pass
    
    # 回零
    def return_zero(self):
        self.set__position_mode()
        # self.__I1_data = self.__I1_error
        self.__T2_data = self.__T2_error
        self.__T3_data = self.__T3_error
        self.__I4_data = self.__I4_error
        self.__T5_data = self.__T5_error
        self.__T6_data = self.__T6_error
        # self.__I7_data = self.__I7_error
        self.__position_mode = True
        pass   