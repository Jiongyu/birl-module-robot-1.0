#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
# import os
# print os.getcwd()
sys.path.append("./src/birl_module_robot/UI/scripts/climbot5d")
import time
from  math import fabs,degrees

from PyQt5.QtWidgets import QWidget,QDesktopWidget
from PyQt5.QtCore import QThread,pyqtSignal
from climbot5d_joint_control import Ui_Climbot5d_joint_control
from climbot5d_variable import Climbot5d_Variable

class Climbot5d_joint_control_func(QWidget,Ui_Climbot5d_joint_control,Climbot5d_Variable):

    # 各关节启动运行信号
    sin_I1_data = pyqtSignal(list)  
    sin_T2_data = pyqtSignal(list)
    sin_T3_data = pyqtSignal(list)
    sin_T4_data = pyqtSignal(list)
    sin_I5_data = pyqtSignal(list)
    sin_G0_data = pyqtSignal(int)
    sin_G6_data = pyqtSignal(int)
    sin_G0_command = pyqtSignal(int)
    sin_G6_command = pyqtSignal(int)

    # 设置零点信号
    sin_set_zero = pyqtSignal()

    # 回零信号
    sin_return_zero = pyqtSignal()

    # 返回上一界面
    sin_return_last_ui = pyqtSignal()
    # 急停
    sin_quick_stop = pyqtSignal()

    def __init__(self,parent=None):
        super(Climbot5d_joint_control_func,self).__init__(parent)
        Climbot5d_Variable.__init__(self)
        self.setupUi(self)
        self.center()

        self.sin_G0_command.connect(self.G0_sent_torque)
        self.sin_G6_command.connect(self.G6_sent_torque)

    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    
    def I1_add_position(self,pressed):
        '''
        push button to add I1 displacement.
        :param pressed:
        :return:
        '''

        if pressed:
            self.pushButton_2.setEnabled(False)
            self.sin_I1_data.emit([1,self.joint_velocity])

        else:
            self.sin_I1_data.emit([1,0])
            self.pushButton_2.setEnabled(True)

    def I1_sub_position(self,pressed):
        '''
        push button to sub I1 displacement.
        :param pressed:
        :return:
        '''

        if pressed:
            self.pushButton_1.setEnabled(False)
            self.sin_I1_data.emit([1, -self.joint_velocity])
            pass

        else:
            self.sin_I1_data.emit([1,0])
            self.pushButton_1.setEnabled(True)
        pass

    def T2_add_position(self,pressed):
        if pressed:
            self.pushButton_7.setEnabled(False)
            self.sin_T2_data.emit([2,self.joint_velocity])

        else:
            self.sin_T2_data.emit([2,0])
            self.pushButton_7.setEnabled(True)
        pass

    def T2_sub_position(self,pressed):
        if pressed:
            self.pushButton_3.setEnabled(False)
            self.sin_T2_data.emit([2,-self.joint_velocity])
        else:
            self.sin_T2_data.emit([2,0])
            self.pushButton_3.setEnabled(True)

    def T3_add_position(self,pressed):
        if pressed:
            self.pushButton_8.setEnabled(False)
            self.sin_T3_data.emit([3,self.joint_velocity])
            pass

        else:
            self.sin_T3_data.emit([3,0])
            self.pushButton_8.setEnabled(True)
        pass

    def T3_sub_position(self,pressed):
        if pressed:
            self.pushButton_4.setEnabled(False)
            self.sin_T3_data.emit([3,-self.joint_velocity])
            pass

        else:
            self.sin_T3_data.emit([3,0])
            self.pushButton_4.setEnabled(True)
        pass

    def T4_add_position(self,pressed):
        if pressed:
            self.pushButton_9.setEnabled(False)
            self.sin_T4_data.emit([4,self.joint_velocity])
            pass

        else:
            self.sin_T4_data.emit([4,0])
            self.pushButton_9.setEnabled(True)
        pass

    def T4_sub_position(self,pressed):
        if pressed:
            self.pushButton_5.setEnabled(False)
            self.sin_T4_data.emit([4,-self.joint_velocity])
            pass

        else:
            self.sin_T4_data.emit([4,0])
            self.pushButton_5.setEnabled(True)
        pass

    def I5_add_position(self,pressed):

        if pressed:
            self.pushButton_10.setEnabled(False)
            self.sin_I5_data.emit([5,self.joint_velocity])
            pass

        else:
            self.sin_I5_data.emit([5,0])
            self.pushButton_10.setEnabled(True)

    def I5_sub_position(self,pressed):

        if pressed:
            self.pushButton_6.setEnabled(False)
            self.sin_I5_data.emit([5,-self.joint_velocity])

            pass

        else:
            self.sin_I5_data.emit([5,0])

            self.pushButton_6.setEnabled(True)
        pass

    def G0_open(self,pressed):
        '''
        push button to open the gripper 0.
        :param pressed:
        :return:
        '''

        if pressed:
            self.pushButton_12.setEnabled(False)
            self.pushButton_13.setEnabled(False)
            self.pushButton_14.setEnabled(False)

            self.sin_G0_command.emit(self.open_gripper)
            self.listWidget.addItem(">>夹持器0打开．")
            pass

        else:

            self.sin_G0_command.emit(self.zero_gripper)
            self.pushButton_12.setEnabled(True)
            self.pushButton_13.setEnabled(True)
            self.pushButton_14.setEnabled(True)

            self.pushButton_1.setEnabled(True)
            self.pushButton_2.setEnabled(True)
            self.pushButton_3.setEnabled(True)
            self.pushButton_4.setEnabled(True)
            self.pushButton_5.setEnabled(True)
            self.pushButton_6.setEnabled(True)
            self.pushButton_7.setEnabled(True)
            self.pushButton_8.setEnabled(True)
            self.pushButton_9.setEnabled(True)
            self.pushButton_10.setEnabled(True)

    def G0_close(self,pressed):

        if pressed:
            self.pushButton_14.setEnabled(False)
            self.pushButton_13.setEnabled(False)
            self.pushButton_11.setEnabled(False)
            self.sin_G0_command.emit(self.close_gripper)
            self.listWidget.addItem(">>夹持器0关闭．")
            pass

        else:
            self.sin_G0_command.emit(self.zero_gripper)
            self.pushButton_14.setEnabled(True)
            self.pushButton_13.setEnabled(True)
            self.pushButton_11.setEnabled(True)

            self.pushButton_1.setEnabled(False)
            self.pushButton_2.setEnabled(False)
            self.pushButton_3.setEnabled(False)
            self.pushButton_4.setEnabled(False)
            self.pushButton_5.setEnabled(False)
            self.pushButton_6.setEnabled(False)
            self.pushButton_7.setEnabled(False)
            self.pushButton_8.setEnabled(False)
            self.pushButton_9.setEnabled(False)
            self.pushButton_10.setEnabled(False)

    def G0_sent_torque(self, data):
        if data == 0:
            self.sin_G0_data.emit(data)
        else:
            self.G0_value = data
            self.sin_G0_data.emit(data)

    def G6_open(self,pressed):

        if pressed:
            self.pushButton_12.setEnabled(False)
            self.pushButton_11.setEnabled(False)
            self.pushButton_14.setEnabled(False)

            self.sin_G6_command.emit(self.open_gripper)
            self.listWidget.addItem(">>夹持器6打开．")
            pass

        else:
            self.sin_G6_command.emit(self.zero_gripper)
            self.pushButton_12.setEnabled(True)
            self.pushButton_11.setEnabled(True)
            self.pushButton_14.setEnabled(True)

            self.pushButton_1.setEnabled(True)
            self.pushButton_2.setEnabled(True)
            self.pushButton_3.setEnabled(True)
            self.pushButton_4.setEnabled(True)
            self.pushButton_5.setEnabled(True)
            self.pushButton_6.setEnabled(True)
            self.pushButton_7.setEnabled(True)
            self.pushButton_8.setEnabled(True)
            self.pushButton_9.setEnabled(True)
            self.pushButton_10.setEnabled(True)

    def G6_close(self,pressed):

        if pressed:
            self.pushButton_13.setEnabled(False)
            self.pushButton_12.setEnabled(False)
            self.pushButton_11.setEnabled(False)
            self.sin_G6_command.emit(self.close_gripper)
            self.listWidget.addItem(">>夹持器6关闭．")
            pass

        else:
            self.sin_G6_command.emit(self.zero_gripper)
            self.pushButton_13.setEnabled(True)
            self.pushButton_12.setEnabled(True)
            self.pushButton_11.setEnabled(True)

            self.pushButton_1.setEnabled(False)
            self.pushButton_2.setEnabled(False)
            self.pushButton_3.setEnabled(False)
            self.pushButton_4.setEnabled(False)
            self.pushButton_5.setEnabled(False)
            self.pushButton_6.setEnabled(False)
            self.pushButton_7.setEnabled(False)
            self.pushButton_8.setEnabled(False)
            self.pushButton_9.setEnabled(False)
            self.pushButton_10.setEnabled(False)
        pass

    def G6_sent_torque(self, data):
        if data == 0:
            self.sin_G6_data.emit(data)
        else:
            self.G6_value = data
            self.sin_G6_data.emit(data)

    def return_last_ui(self):
        '''
        return last ui and sent real robot current data.
        :return:
        '''
        self.sin_return_last_ui.emit()

    def change_velocity(self,data):
        '''
        move the slider to change the joint_velocity.
        :param data:
        :return:
        '''
        self.joint_velocity = data * 0.001  # rad
        self.listWidget.addItem(">>>速度:{}deg/s".format(round(degrees(self.joint_velocity),3)))
        print self.joint_velocity

    def set_zero(self):
        self.sin_set_zero.emit()
        self.listWidget.addItem('>>>已设置零点.')
        pass

    def return_zero(self):
        self.sin_return_zero.emit()
        pass

    def quick_stop(self):
        self.sin_quick_stop.emit()
