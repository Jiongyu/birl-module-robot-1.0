#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
from os.path import exists
sys.path.append("./src/birl_module_robot/UI/scripts/climbot5d")
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

from wheel_bipdal_7d_data_show_func import Wheel_bipedal_7d_data_show_func

from wheel_bipedal_7d_func import Wheel_bipedal_7d_func

from wheel_bipedal_7d_mode_set import Ui_wheel_bipedal_7d_mode_set

from wheel_transmit_joint_data import Thread_transmit_joint_data_wheel

sys.path.append("./src/birl_module_robot/UI/scripts")
# Absolute address
sys.path.append("/home/tan/ros/module_robot/src/birl_module_robot/canopen_communication/modular")

# from os import getcwd,path
# print getcwd()

class wheel_bipedal_7d_mode_set_func(QWidget,Ui_wheel_bipedal_7d_mode_set):
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
    # 回零位使用
    sin_return_zero = pyqtSignal()
    # 示教记录使用
    sin_get_teach_data = pyqtSignal(list)

    def __init__(self,simulation=False):
        super(wheel_bipedal_7d_mode_set_func,self).__init__()
        self.setupUi(self)
        self.center()
        self.simulation = simulation

        
        if not self.simulation:

            # 底层通信类
            self.sent_joint_data = Thread_transmit_joint_data_wheel()
            # 初始化成功,错误信号
            self.sent_joint_data.sin_init_error.connect(self.init_error)
            self.sent_joint_data.sin_init_success.connect(self.init_success)
            # 连接反馈
            self.sent_joint_data.sin_joint_control_actual_joint_data.connect(self.joint_control_feedback_data)
            self.sent_joint_data.sin_offline_data_actual_joint_data.connect(self.offline_control_feedback_data)
            # 模式设置连接
            self.sin_open_velocity_mode.connect(self.sent_joint_data.set_velocity_mode)
            self.sin_open_position_mode.connect(self.sent_joint_data.set__position_mode)
            # 离线数据窗口暂停连接
            self.sin_pause_sent_offline_data.connect(self.sent_joint_data.pause_run)
            # 各窗口发送数据连接
            self.sin_joint_data.connect(self.sent_joint_data.joint_sent_data)
            self.sin_offline_data.connect(self.sent_joint_data.offline_sent_data)
            # 设置零点连接
            self.sin_set_zero_position.connect(self.sent_joint_data.get_zero_position)
            # 停止连接
            self.sin_stop_robot_command.connect(self.sent_joint_data.if_stop)
            self.sin_quick_stop.connect(self.sent_joint_data.if_quick_stop)
            # 回零连接
            self.sin_return_zero.connect(self.sent_joint_data.return_zero)

            self.sent_joint_data.start()

        self.I1_value = 0
        self.T2_value = 0
        self.T3_value = 0
        self.I4_value = 0
        self.T5_value = 0
        self.T6_value = 0
        self.I7_value = 0

        # 设置零点使用
        self.I1_error = 0
        self.T2_error = 0
        self.T3_error = 0
        self.I4_error = 0
        self.T5_error = 0
        self.T6_error = 0
        self.I7_error = 0

    # 关节空间控制窗口
    def joint_control(self):
        """
        open windows of joint control.
        """

        # Initialize the windows of joint control.
        self.windows_joint_control = Wheel_bipedal_7d_func()
        self.windows_joint_control.sin_return_last_ui.connect(self.close_joint_control)
        self.windows_joint_control.sin_I1_data.connect(self.sent_I1_data)
        self.windows_joint_control.sin_T2_data.connect(self.sent_T2_data)
        self.windows_joint_control.sin_T3_data.connect(self.sent_T3_data)
        self.windows_joint_control.sin_I4_data.connect(self.sent_I4_data)
        self.windows_joint_control.sin_T5_data.connect(self.sent_T5_data)
        self.windows_joint_control.sin_T6_data.connect(self.sent_T6_data)
        self.windows_joint_control.sin_I7_data.connect(self.sent_I7_data)

        self.windows_joint_control.sin_quick_stop.connect(self.quick_stop)
        self.windows_joint_control.sin_set_zero.connect(self.set_zero)
        self.windows_joint_control.sin_return_zero.connect(self.return_zero)

        self.windows_joint_control.lineEdit_2.setText("{0}".format(round(degrees(self.I1_value - self.I1_error),3)))
        self.windows_joint_control.lineEdit_3.setText("{0}".format(round(degrees(self.T2_value - self.T2_error),3)))
        self.windows_joint_control.lineEdit_5.setText("{0}".format(round(degrees(self.T3_value - self.T3_error),3)))
        self.windows_joint_control.lineEdit_4.setText("{0}".format(round(degrees(self.I4_value - self.I4_error),3)))
        self.windows_joint_control.lineEdit_6.setText("{0}".format(round(degrees(self.T5_value - self.T5_error),3)))
        self.windows_joint_control.lineEdit_7.setText("{0}".format(round(degrees(self.T6_value - self.T6_error),3)))
        self.windows_joint_control.lineEdit_8.setText("{0}".format(round(degrees(self.I7_value - self.I7_error),3)))


        self.form.hide()
        self.windows_joint_control.show()
        
        if not self.simulation:
            # self.sin_open_velocity_mode.emit()
            pass
    
    # 离线数据窗口
    def data_show(self):
        '''
        open the windows of the data show.
        :return:
        '''
        # Initialize the windows of data show.
        
        self.windows_data_show = Wheel_bipedal_7d_data_show_func()
        self.sin_sent_data_to_show_monitor.connect(self.windows_data_show.sent_data_monitor)
        self.windows_data_show.sin_return_last_ui.connect(self.close_data_show)
        self.windows_data_show.sin_joint_data.connect(self.data_show_sent_joint_data)
        self.windows_data_show.sin_quick_stop.connect(self.quick_stop)
        self.windows_data_show.sin_pause_data_show.connect(self.data_show_pause)

        self.form.hide()
        self.windows_data_show.show()
        # self.sin_open_velocity_mode.emit()

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
        self.form.show()
        del self.windows_joint_control       
    
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
        pass

    def sent_T2_data(self,data):
        if not self.simulation:
            self.sin_joint_data.emit(data)
        pass

    def sent_T3_data(self,data):
        if not self.simulation:
            self.sin_joint_data.emit(data)
        pass      

    def sent_I4_data(self,data):
        if not self.simulation:
            self.sin_joint_data.emit(data)
        pass

    def sent_T5_data(self,data):
        if not self.simulation:
            self.sin_joint_data.emit(data)
        pass

    def sent_T6_data(self,data):
        if not self.simulation:
            self.sin_joint_data.emit(data)
        pass

    def sent_I7_data(self,data):
        if not self.simulation:
            self.sin_joint_data.emit(data)
        pass

    # 发送离线数据
    def data_show_sent_joint_data(self,data):
        '''
        sent joint data to motor in the windows of data show.
        :param data:
        :return:
        '''
        if not self.simulation:
            self.sin_offline_data.emit(data)
        # print data
        pass
    
    # 离线数据暂停
    def data_show_pause(self, data):
        self.sin_pause_sent_offline_data.emit(data) 
        # print "pause"
        pass
    
    # 设置零点
    def set_zero(self):

        self.I1_error = self.I1_value
        self.T2_error = self.T2_value
        self.T3_error = self.T3_value
        self.I4_error = self.I4_value
        self.T5_error = self.T5_value
        self.T6_error = self.T6_value
        self.I7_error = self.I7_value
        
        self.windows_joint_control.lineEdit_2.setText("{0}".format(round(degrees(self.I1_value - self.I1_error), 3)))
        self.windows_joint_control.lineEdit_3.setText("{0}".format(round(degrees(self.T2_value - self.T2_error), 3)))
        self.windows_joint_control.lineEdit_5.setText("{0}".format(round(degrees(self.T3_value - self.T3_error), 3)))
        self.windows_joint_control.lineEdit_4.setText("{0}".format(round(degrees(self.I4_value - self.I4_error), 3)))            
        self.windows_joint_control.lineEdit_6.setText("{0}".format(round(degrees(self.T5_value - self.T5_error), 3)))
        self.windows_joint_control.lineEdit_7.setText("{0}".format(round(degrees(self.T6_value - self.T6_error), 3)))
        self.windows_joint_control.lineEdit_8.setText("{0}".format(round(degrees(self.I7_value - self.I7_error), 3)))

        self.sin_set_zero_position.emit([self.I1_error,self.T2_error,self.T3_error,self.I4_error,self.T5_error,self.T6_error,self.I7_error])
    
    # 接收关节数据反馈
    def joint_control_feedback_data(self,data):
        try:
            self.I1_value = data[0]
            self.T2_value = data[1]
            self.T3_value = data[2] 
            self.I4_value = data[3]
            self.T5_value = data[4]
            self.T6_value = data[5]
            self.I7_value = data[6]

            self.windows_joint_control.lineEdit_2.setText("{0}".format(round(degrees(self.I1_value - self.I1_error),3)))
            self.windows_joint_control.lineEdit_3.setText("{0}".format(round(degrees(self.T2_value - self.T2_error),3)))
            self.windows_joint_control.lineEdit_5.setText("{0}".format(round(degrees(self.T3_value - self.T3_error),3)))
            self.windows_joint_control.lineEdit_4.setText("{0}".format(round(degrees(self.I4_value - self.I4_error),3)))
            self.windows_joint_control.lineEdit_6.setText("{0}".format(round(degrees(self.T5_value - self.T5_error),3)))
            self.windows_joint_control.lineEdit_7.setText("{0}".format(round(degrees(self.T6_value - self.T6_error),3)))
            self.windows_joint_control.lineEdit_8.setText("{0}".format(round(degrees(self.I7_value - self.I7_error),3)))
        except:
            pass

    # 接受离线数据反馈
    def offline_control_feedback_data(self,data):
        try:
            self.sin_sent_data_to_show_monitor.emit(data)
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

    
    # 回零位 
    def return_zero(self):
        if not self.simulation:
            self.sin_return_zero.emit()
        pass