#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
# import os
# print os.getcwd()
sys.path.append("./src/birl_module_robot/UI/scripts/arm5d")
import time
from  math import fabs,degrees

from PyQt5.QtWidgets import QWidget,QDesktopWidget
from PyQt5.QtCore import QThread,pyqtSignal
from arm5d_joint_control import Ui_arm5d_joint_control

class Arm5d_joint_control_func(QWidget,Ui_arm5d_joint_control):
    
    # 各关节启动运行信号
    sin_I1_data = pyqtSignal(list)  
    sin_T2_data = pyqtSignal(list)
    sin_T3_data = pyqtSignal(list)
    sin_i4_data = pyqtSignal(list)
    sin_t5_data = pyqtSignal(list)

    # 设置零点信号
    sin_set_zero = pyqtSignal()

    # 回零信号
    sin_return_zero = pyqtSignal()

    # 返回上一界面
    sin_return_last_ui = pyqtSignal()

    # 急停
    sin_quick_stop = pyqtSignal()

    def __init__(self,parent=None):
        super(Arm5d_joint_control_func,self).__init__(parent)
        self.setupUi(self)
        self.center()
        self.joint_velocity = 0.02

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
            self.listWidget.addItem("I1+")

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
            self.listWidget.addItem("I1-")

            pass

        else:
            self.sin_I1_data.emit([1,0])
            self.pushButton_1.setEnabled(True)
        pass

    def T2_add_position(self,pressed):
        if pressed:
            self.pushButton_7.setEnabled(False)
            self.sin_T2_data.emit([2,self.joint_velocity])
            self.listWidget.addItem("T2+")


        else:
            self.sin_T2_data.emit([2,0])
            self.pushButton_7.setEnabled(True)

        pass

    def T2_sub_position(self,pressed):
        if pressed:
            self.pushButton_3.setEnabled(False)
            self.sin_T2_data.emit([2,-self.joint_velocity])
            self.listWidget.addItem("T2-")

        else:
            self.sin_T2_data.emit([2,0])
            self.pushButton_3.setEnabled(True)

    def T3_add_position(self,pressed):
        if pressed:
            self.pushButton_8.setEnabled(False)
            self.sin_T3_data.emit([3,self.joint_velocity])
            self.listWidget.addItem("T3+")

            pass

        else:
            self.sin_T3_data.emit([3,0])
            self.pushButton_8.setEnabled(True)
        pass

    def T3_sub_position(self,pressed):
        if pressed:
            self.pushButton_4.setEnabled(False)
            self.sin_T3_data.emit([3,-self.joint_velocity])
            self.listWidget.addItem("T3-")

            pass

        else:
            self.sin_T3_data.emit([3,0])
            self.pushButton_4.setEnabled(True)
        pass

    def i4_add_position(self,pressed):
        if pressed:
            self.pushButton_9.setEnabled(False)
            self.sin_i4_data.emit([4,self.joint_velocity])
            self.listWidget.addItem("i4+")

            pass

        else:
            self.sin_i4_data.emit([4,0])
            self.pushButton_9.setEnabled(True)

        pass

    def i4_sub_position(self,pressed):
        if pressed:
            self.pushButton_5.setEnabled(False)
            self.sin_i4_data.emit([4,-self.joint_velocity])
            self.listWidget.addItem("i4-")

            pass

        else:
            self.sin_i4_data.emit([4,0])
            self.pushButton_5.setEnabled(True)
        pass

    def t5_add_position(self,pressed):

        if pressed:
            self.pushButton_10.setEnabled(False)
            self.sin_t5_data.emit([5,self.joint_velocity])
            self.listWidget.addItem("t5+")

            pass

        else:
            self.sin_t5_data.emit([5,0])
            self.pushButton_10.setEnabled(True)

    def t5_sub_position(self,pressed):

        if pressed:
            self.pushButton_6.setEnabled(False)
            self.sin_t5_data.emit([5,-self.joint_velocity])
            self.listWidget.addItem("t5-")

            pass

        else:
            self.sin_t5_data.emit([5,0])

            self.pushButton_6.setEnabled(True)
        pass

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
        # print self.joint_velocity

    def set_zero(self):
        self.sin_set_zero.emit()
        self.listWidget.addItem('>>>已设置零点.')

    def return_zero(self):
        self.sin_return_zero.emit()
        pass

    def quick_stop(self):
        self.sin_quick_stop.emit()