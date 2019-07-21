#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
from os.path import exists
sys.path.append("./src/birl_module_robot/UI/scripts/arm5d")
# from os import getcwd,path
# print getcwd()

import time
import traceback
from math import degrees, radians


import rospy
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QDesktopWidget, QMessageBox, QWidget
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from arm5d_data_show_func import Arm5d_data_show_func
from arm5d_descartes_control_func import Arm5d_descartes_control_func
from arm5d_joint_control_func import Arm5d_joint_control_func
from arm5d_mode_set import Ui_arm5d_mode_set

from arm5d_transmit_joint_data  import Thread_transmit_joint_data_arm

sys.path.append("./src/birl_module_robot/UI/scripts")
# Absolute address
sys.path.append("/home/tan/ros/module_robot/src/birl_module_robot/canopen_communication/modular")

# from os import getcwd,path
# print getcwd()

class Arm5d_mode_set_func(QWidget,Ui_arm5d_mode_set):
    # 关闭当前界面信号
    sin_close =pyqtSignal()
    # 控制运行模式信号
    sin_open_position_mode = pyqtSignal()
    sin_open_velocity_mode = pyqtSignal()
    # 控制数据信号
    sin_joint_data = pyqtSignal(list)
    sin_descartes_data = pyqtSignal(list)
    sin_offline_data = pyqtSignal(list)
    #暂停离线数据信号
    sin_pause_sent_offline_data = pyqtSignal(bool)

    sin_stop_robot_command = pyqtSignal()
    # 急停信号
    sin_quick_stop = pyqtSignal()
    # 离线数据显示信号
    sin_sent_data_to_show_monitor = pyqtSignal(list)
    # 设置零点信号
    sin_set_zero_position = pyqtSignal(list)
    # 末端笛卡尔点初始化信号
    sin_descartes_init_data = pyqtSignal(list)
    # 回零位使用
    sin_return_zero = pyqtSignal()
    # 示教记录使用
    sin_get_teach_data = pyqtSignal(list)

    def __init__(self,simulation=False):
        super(Arm5d_mode_set_func,self).__init__()
        self.setupUi(self)
        self.center()
        self.simulation = simulation

        
        rospy.init_node('Arm5d_control')
        # this publisher was to sent joint points by positive algorithm to getting the descartes datas.
        self.pub_wjc = rospy.Publisher('arm5d_Joint_point', Float64MultiArray, queue_size=5)
        # this subscriber was to receive the descartes data.
        self.sub_wjc = rospy.Subscriber('arm5d_Positive_solution', Float64MultiArray,self.positive_solution_callback)
        self.positive_solution = Float64MultiArray()
        self.joint_point = Float64MultiArray()
        self.positive_solution.data = [0,0,0,0,0,0]
        self.joint_point.data = [0,0,0,0,0,0]

        # self.pub_simulation = rospy.Publisher('/climbot5d_simulation_joint_command',JointState,queue_size=30)
        # self.sub_simulation = rospy.Subscriber('JointState',JointState,self.simulation_callback)
        # self.simulation_joint_command = JointState()
        # self.simulation_joint_command.name = ["G0_Joint", "I1_Joint", "T2_Joint", "T3_Joint", "i4_Joint", "t5_Joint", "G6_Joint"]
        # self.simulation_joint_state = JointState()
        
        if not self.simulation:

            # 底层通信类
            self.sent_joint_data = Thread_transmit_joint_data_arm()
            # 初始化成功,错误信号
            self.sent_joint_data.sin_init_error.connect(self.init_error)
            self.sent_joint_data.sin_init_success.connect(self.init_success)
            # 连接反馈
            self.sent_joint_data.sin_joint_control_actual_joint_data.connect(self.joint_control_feedback_data)
            self.sent_joint_data.sin_descartes_control_actual_joint_data.connect(self.descartes_control_feedback_data)
            self.sent_joint_data.sin_offline_data_actual_joint_data.connect(self.offline_control_feedback_data)
            # 模式设置连接
            self.sin_open_velocity_mode.connect(self.sent_joint_data.set_velocity_mode)
            self.sin_open_position_mode.connect(self.sent_joint_data.set__position_mode)
            # 离线数据窗口暂停连接
            self.sin_pause_sent_offline_data.connect(self.sent_joint_data.pause_run)
            # 各窗口发送数据连接
            self.sin_joint_data.connect(self.sent_joint_data.joint_sent_data)
            self.sin_descartes_data.connect(self.sent_joint_data.descartes_sent_data)
            self.sin_offline_data.connect(self.sent_joint_data.offline_sent_data)
            # 设置零点连接
            self.sin_set_zero_position.connect(self.sent_joint_data.get_zero_position)
            # 末端笛卡尔连接
            self.sin_descartes_init_data.connect(self.sent_joint_data.get_init_descartes_point)
            # 停止连接
            self.sin_stop_robot_command.connect(self.sent_joint_data.if_stop)
            self.sin_quick_stop.connect(self.sent_joint_data.if_quick_stop)
            # 回零连接
            self.sin_return_zero.connect(self.sent_joint_data.return_zero)

            self.sent_joint_data.start()

        self.I1_value = 0
        self.T2_value = 0
        self.T3_value = 0
        self.i4_value = 0
        self.t5_value = 0

        self.Descartes_X = 1.0467
        self.Descartes_Y = 0
        self.Descartes_Z = 0
        self.Descartes_RX = 3.141592654
        self.Descartes_RY = 0
        self.Descartes_RZ = 0

        # 设置零点使用
        self.I1_error = 0
        self.T2_error = 0
        self.T3_error = 0
        self.i4_error = 0
        self.t5_error = 0
        
        self.X_error = 0
        self.Y_error = 0
        self.Z_error = 0
        self.RX_error = 0
        self.RY_error = 0
        self.RZ_error = 0

        # 存储示教数据
        self.__teach_file_index = 0 
        self.__teach_file = "./src/birl_module_robot/UI/teach_data/teach_data_{0}.txt".format(self.__teach_file_index)


    # def simulation_callback(set,msg):
    #     pass
    
    # 正解回调函数
    def positive_solution_callback(self, msg):
        self.positive_solution = msg

    # 关节空间控制窗口
    def joint_control(self):
        """
        open windows of joint control.
        """

        # Initialize the windows of joint control.
        self.windows_joint_control = Arm5d_joint_control_func()
        self.windows_joint_control.sin_return_last_ui.connect(self.close_joint_control)
        self.windows_joint_control.sin_I1_data.connect(self.sent_I1_data)
        self.windows_joint_control.sin_T2_data.connect(self.sent_T2_data)
        self.windows_joint_control.sin_T3_data.connect(self.sent_T3_data)
        self.windows_joint_control.sin_i4_data.connect(self.sent_i4_data)
        self.windows_joint_control.sin_t5_data.connect(self.sent_t5_data)
        self.windows_joint_control.sin_quick_stop.connect(self.quick_stop)
        self.windows_joint_control.sin_set_zero.connect(self.set_zero)
        self.windows_joint_control.sin_return_zero.connect(self.return_zero)

        self.windows_joint_control.lineEdit_2.setText("{0}".format(round(degrees(self.I1_value - self.I1_error),3)))
        self.windows_joint_control.lineEdit_3.setText("{0}".format(round(degrees(self.T2_value - self.T2_error),3)))
        self.windows_joint_control.lineEdit_5.setText("{0}".format(round(degrees(self.T3_value - self.T3_error),3)))
        self.windows_joint_control.lineEdit_4.setText("{0}".format(round(degrees(self.i4_value - self.i4_error),3)))
        self.windows_joint_control.lineEdit_6.setText("{0}".format(round(degrees(self.t5_value - self.t5_error),3)))

        self.form.hide()
        self.windows_joint_control.show()
        
        if not self.simulation:
            self.sin_open_velocity_mode.emit()
    
    # 笛卡尔空间控制窗口
    def decartes_control(self):
        """
        open the windows of descartes control.
        :return:
        """

        # Initialize the windows of descartes control.
        self.windows_decartes_control = Arm5d_descartes_control_func()
        self.windows_decartes_control.sin_return_last_ui.connect(self.close_decartes_control)
        self.windows_decartes_control.sin_X_data.connect(self.sent_X)
        self.windows_decartes_control.sin_Y_data.connect(self.sent_Y)
        self.windows_decartes_control.sin_Z_data.connect(self.sent_Z)
        self.windows_decartes_control.sin_RX_data.connect(self.sent_RX)
        self.windows_decartes_control.sin_RY_data.connect(self.sent_RY)
        self.windows_decartes_control.sin_RZ_data.connect(self.sent_RZ)

        self.sin_get_teach_data.connect(self.windows_decartes_control.receive_teach_data)
        
        if not self.simulation:
            self.windows_decartes_control.sin_quick_stop.connect(self.quick_stop)

        self.form.hide()
        self.windows_decartes_control.show()
        self.sin_open_position_mode.emit()

        self.windows_decartes_control.lineEdit_2.setText('{0}'.format(round(degrees(self.I1_value - self.I1_error),3)))
        self.windows_decartes_control.lineEdit_3.setText('{0}'.format(round(degrees(self.T2_value - self.T2_error),3)))
        self.windows_decartes_control.lineEdit_5.setText('{0}'.format(round(degrees(self.T3_value - self.T3_error),3)))
        self.windows_decartes_control.lineEdit_4.setText('{0}'.format(round(degrees(self.i4_value - self.i4_error),3)))
        self.windows_decartes_control.lineEdit_6.setText('{0}'.format(round(degrees(self.t5_value - self.t5_error),3)))

        self.windows_decartes_control.lineEdit_9.setText('{0}'.format(1000*round(self.Descartes_X - self.X_error,4)))
        self.windows_decartes_control.lineEdit_13.setText('{0}'.format(1000*round(self.Descartes_Y - self.Y_error,4)))
        self.windows_decartes_control.lineEdit_12.setText('{0}'.format(1000*round(self.Descartes_Z - self.Z_error),4))
        self.windows_decartes_control.lineEdit_10.setText('{0}'.format(round(degrees(self.Descartes_RX - self.RX_error),3)))
        self.windows_decartes_control.lineEdit_8.setText('{0}'.format(round(degrees(self.Descartes_RY - self.RY_error),3)))
        self.windows_decartes_control.lineEdit_11.setText('{0}'.format(round(degrees(self.Descartes_RZ - self.RZ_error),3)))
    
    # 离线数据窗口
    def data_show(self):
        '''
        open the windows of the data show.
        :return:
        '''
        # Initialize the windows of data show.
        
        self.windows_data_show = Arm5d_data_show_func()
        self.sin_sent_data_to_show_monitor.connect(self.windows_data_show.sent_data_monitor)
        self.windows_data_show.sin_return_last_ui.connect(self.close_data_show)
        self.windows_data_show.sin_joint_data.connect(self.data_show_sent_joint_data)
        self.windows_data_show.sin_quick_stop.connect(self.quick_stop)
        self.windows_data_show.sin_pause_data_show.connect(self.data_show_pause)
        self.form.hide()
        self.windows_data_show.show()
        self.sin_open_position_mode.emit()

    # 关闭离线数据窗口
    def close_data_show(self):
        '''
        close the windows of the data show.
        :return:
        '''
        self.windows_data_show.close()
        self.form.show()
        del self.windows_data_show
        pass

    # 关闭关节空间控制窗口
    def close_joint_control(self):
        '''
        close windows of joint control.
        :param data: all joint data.
        :return:
        '''
        self.windows_joint_control.close()
        self.__get_positive_value()
        self.form.show()
        self.sin_descartes_init_data.emit([self.Descartes_X,self.Descartes_Y,self.Descartes_Z,self.Descartes_RX,self.Descartes_RY,self.Descartes_RZ])
        del self.windows_joint_control       

    # 关闭笛卡尔空间控制窗口 
    def close_decartes_control(self):
        '''
        open the windows of descartes control.
        :return:
        '''
        self.windows_decartes_control.close()
        self.form.show()
        del self.windows_decartes_control
        pass
    
    # 返回上一窗口
    def return_last_ui(self):
        '''
        return last ui.
        :return:
        '''
        if not self.simulation:
            self.sin_stop_robot_command.emit()
        self.sin_close.emit()
        pass
    
    # 窗口设置为屏幕中心
    def center(self):
        """
        put the ui in the center of current window.
        :return:
        """
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
    
    # 关节空间,发送关节数据
    def sent_I1_data(self,data):
        if not self.simulation:
            self.sin_joint_data.emit(data)
        
        print data

    def sent_T2_data(self,data):
        if not self.simulation:
            self.sin_joint_data.emit(data)   
        # print data

    def sent_T3_data(self,data):
        if not self.simulation:
            self.sin_joint_data.emit(data)
        # print data

    def sent_i4_data(self,data):
        if not self.simulation:
            self.sin_joint_data.emit(data)
        # print data

    def sent_t5_data(self,data):
        if not self.simulation:
            self.sin_joint_data.emit(data)
        # print data
    

    # 发送末端笛卡尔点数据
    def sent_X(self,data):
        # print data
        self.sin_descartes_data.emit([1,data])

    def sent_Y(self,data):
        # print data
        self.sin_descartes_data.emit([2,data])

    def sent_Z(self,data):
        # print data
        self.sin_descartes_data.emit([3,data])

    def sent_RX(self,data):
        # print data
        self.sin_descartes_data.emit([4,data])

    def sent_RY(self,data):
        # print data
        self.sin_descartes_data.emit([5,data])

    def sent_RZ(self,data):
        # print data
        self.sin_descartes_data.emit([6,data])
        
    # 发送离线数据
    def data_show_sent_joint_data(self,data):
        '''
        sent joint data to motor in the windows of data show.
        :param data:
        :return:
        '''
        # print data
        if not self.simulation:
            self.sin_offline_data.emit(data)
    
    # 离线数据暂停
    def data_show_pause(self, data):
        self.sin_pause_sent_offline_data.emit(data) 
        pass
    
    # 设置零点
    def set_zero(self):

        self.I1_error = self.I1_value
        self.T2_error = self.T2_value
        self.T3_error = self.T3_value
        self.i4_error = self.i4_value
        self.t5_error = self.t5_value

        self.joint_point.data = [self.I1_value,self.T2_value,self.T3_value,self.i4_value,self.t5_value]
        self.pub_wjc.publish(self.joint_point)
        self.windows_joint_control.lineEdit_2.setText("{0}".format(round(degrees(self.I1_value - self.I1_error), 3)))
        self.windows_joint_control.lineEdit_3.setText("{0}".format(round(degrees(self.T2_value - self.T2_error), 3)))
        self.windows_joint_control.lineEdit_5.setText("{0}".format(round(degrees(self.T3_value - self.T3_error), 3)))
        self.windows_joint_control.lineEdit_4.setText("{0}".format(round(degrees(self.i4_value - self.i4_error),3)))            
        self.windows_joint_control.lineEdit_6.setText("{0}".format(round(degrees(self.t5_value - self.t5_error),3)))

        self.X_error = self.positive_solution.data[0] - 1.0467
        self.Y_error = self.positive_solution.data[1]
        self.Z_error = self.positive_solution.data[2]
        self.RX_error = self.positive_solution.data[3] - 3.141592654
        self.RY_error = self.positive_solution.data[4]
        self.RZ_error = self.positive_solution.data[5] 

        self.sin_set_zero_position.emit([self.I1_error,self.T2_error,self.T3_error,self.i4_error,self.t5_error])
    
    # 接收关节数据反馈
    def joint_control_feedback_data(self,data):
        try:
            self.I1_value = data[0]
            self.T2_value = data[1]
            self.T3_value = data[2] 
            self.i4_value = data[3]
            self.t5_value = data[4]
            self.windows_joint_control.lineEdit_2.setText("{0}".format(round(degrees(self.I1_value - self.I1_error),3)))
            self.windows_joint_control.lineEdit_3.setText("{0}".format(round(degrees(self.T2_value - self.T2_error),3)))
            self.windows_joint_control.lineEdit_5.setText("{0}".format(round(degrees(self.T3_value - self.T3_error),3)))
            self.windows_joint_control.lineEdit_4.setText("{0}".format(round(degrees(self.i4_value - self.i4_error),3)))
            self.windows_joint_control.lineEdit_6.setText("{0}".format(round(degrees(self.t5_value - self.t5_error),3)))
        except:
            pass

    # 接受离线数据反馈
    def offline_control_feedback_data(self,data):
        try:
            self.I1_value ,self.T2_value ,self.T3_value ,self.i4_value ,self.t5_value  = data
            self.__get_positive_value()
            self.sin_sent_data_to_show_monitor.emit([self.I1_value ,self.T2_value ,self.T3_value ,self.i4_value ,self.t5_value, \
                                                    self.Descartes_X,self.Descartes_Y,self.Descartes_Z,self.Descartes_RY,self.Descartes_RY,self.Descartes_RZ])
        
        except:
            pass

    # 接收笛卡尔数据反馈
    def descartes_control_feedback_data(self,data):
        try:
            self.I1_value = data[0]
            self.T2_value = data[1]
            self.T3_value = data[2] 
            self.i4_value = data[3]
            self.t5_value = data[4]

            self.windows_decartes_control.lineEdit_2.setText("{0}".format(round(degrees(self.I1_value - self.I1_error),3)))
            self.windows_decartes_control.lineEdit_3.setText("{0}".format(round(degrees(self.T2_value - self.T2_error),3)))
            self.windows_decartes_control.lineEdit_5.setText("{0}".format(round(degrees(self.T3_value - self.T3_error),3)))
            self.windows_decartes_control.lineEdit_4.setText("{0}".format(round(degrees(self.i4_value - self.i4_error),3)))
            self.windows_decartes_control.lineEdit_6.setText("{0}".format(round(degrees(self.t5_value - self.t5_error),3)))

            self.__get_positive_value()
            self.windows_decartes_control.lineEdit_9.setText('{0}'.format(1000*round(self.Descartes_X - self.X_error,4)))
            self.windows_decartes_control.lineEdit_13.setText('{0}'.format(1000*round(self.Descartes_Y - self.Y_error,4)))
            self.windows_decartes_control.lineEdit_12.setText('{0}'.format(1000*round(self.Descartes_Z - self.Z_error),4))
            self.windows_decartes_control.lineEdit_10.setText('{0}'.format(round((self.Descartes_RX - self.RX_error),3)))
            self.windows_decartes_control.lineEdit_8.setText('{0}'.format(round((self.Descartes_RY - self.RY_error),3)))
            self.windows_decartes_control.lineEdit_11.setText('{0}'.format(round((self.Descartes_RZ - self.RZ_error),3)))
        except:
            pass

    # 初始化成功
    def init_success(self):
        self.__box_1 = QMessageBox(QMessageBox.Warning, "提示", "canopen通信初始化成功！！")
        self.__box_1.addButton(self.tr("确定"), QMessageBox.YesRole)
        self.__box_1.exec_()

    # 初始化错误
    def init_error(self):
        self.__box_1 = QMessageBox(QMessageBox.Warning, "错误", "canopen通信初始化错误！！\n无法读取can信息．\n请检查是否连接正确．")
        self.__box_1.addButton(self.tr("确定"), QMessageBox.YesRole)
        self.__box_1.exec_()
        self.return_last_ui()
    
    # 槽函数 急停
    def quick_stop(self):
        self.sin_quick_stop.emit()

    # 获取正解
    def __get_positive_value(self):

        self.joint_point.data = [self.I1_value,self.T2_value,self.T3_value,self.i4_value,self.t5_value]
        self.pub_wjc.publish(self.joint_point)
        self.Descartes_X = self.positive_solution.data[0]
        self.Descartes_Y = self.positive_solution.data[1]
        self.Descartes_Z = self.positive_solution.data[2]
        self.Descartes_RX = self.positive_solution.data[3]
        self.Descartes_RY = self.positive_solution.data[4]
        self.Descartes_RZ = self.positive_solution.data[5]
        # print [self.Descartes_X,self.Descartes_Y,self.Descartes_Z,self.Descartes_RY,self.Descartes_RY,self.Descartes_RZ]

    # 回零位 
    def return_zero(self):
        if not self.simulation:
            self.sin_return_zero.emit()
        pass