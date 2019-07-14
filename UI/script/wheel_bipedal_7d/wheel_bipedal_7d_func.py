#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
# import os
# print os.getcwd()
sys.path.append("./src/birl_module_robot/UI/scripts/wheel_bipedal_7d")
import time
from  math import fabs,degrees

from PyQt5.QtWidgets import QWidget,QDesktopWidget
from PyQt5.QtCore import QThread,pyqtSignal
from wheel_bipedal_7d import Ui_wheel_bipedal_7d

class Wheel_bipedal_7d_func(QWidget,Ui_wheel_bipedal_7d):

    # 各关节启动运行信号
    sin_I1_data = pyqtSignal(list)  
    sin_T2_data = pyqtSignal(list)
    sin_T3_data = pyqtSignal(list)
    sin_I4_data = pyqtSignal(list)
    sin_T5_data = pyqtSignal(list)
    sin_T6_data = pyqtSignal(list)
    sin_I7_data = pyqtSignal(list)

    # 设置零点信号
    sin_set_zero = pyqtSignal()

    # 回零信号
    sin_return_zero = pyqtSignal()

    # 返回上一界面
    sin_return_last_ui = pyqtSignal()
    # 急停
    sin_quick_stop = pyqtSignal()


    def __init__(self,parent=None):
        super(Wheel_bipedal_7d_func,self).__init__(parent)
        self.joint_velocity = 0.02

        self.setupUi(self)
        self.center()

    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    
    def I1_add(self,pressed):
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

    def I1_sub(self,pressed):
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

    def T2_add(self,pressed):
        if pressed:
            self.pushButton_7.setEnabled(False)
            self.sin_T2_data.emit([2,self.joint_velocity])

        else:
            self.sin_T2_data.emit([2,0])
            self.pushButton_7.setEnabled(True)
        pass

    def T2_sub(self,pressed):
        if pressed:
            self.pushButton_3.setEnabled(False)
            self.sin_T2_data.emit([2,-self.joint_velocity])
        else:
            self.sin_T2_data.emit([2,0])
            self.pushButton_3.setEnabled(True)

    def T3_add(self,pressed):
        if pressed:
            self.pushButton_8.setEnabled(False)
            self.sin_T3_data.emit([3,self.joint_velocity])
            pass

        else:
            self.sin_T3_data.emit([3,0])
            self.pushButton_8.setEnabled(True)
        pass

    def T3_sub(self,pressed):
        if pressed:
            self.pushButton_4.setEnabled(False)
            self.sin_T3_data.emit([3,-self.joint_velocity])
            pass

        else:
            self.sin_T3_data.emit([3,0])
            self.pushButton_4.setEnabled(True)
        pass

    def I4_add(self,pressed):
        if pressed:
            self.pushButton_9.setEnabled(False)
            self.sin_I4_data.emit([4,self.joint_velocity])
            pass

        else:
            self.sin_I4_data.emit([4,0])
            self.pushButton_9.setEnabled(True)
        pass

    def I4_sub(self,pressed):
        if pressed:
            self.pushButton_5.setEnabled(False)
            self.sin_I4_data.emit([4,-self.joint_velocity])
            pass

        else:
            self.sin_I4_data.emit([4,0])
            self.pushButton_5.setEnabled(True)
        pass

    def T5_add(self,pressed):
        if pressed:
            self.pushButton_10.setEnabled(False)
            self.sin_T5_data.emit([5,self.joint_velocity])
            pass

        else:
            self.sin_T5_data.emit([5,0])
            self.pushButton_10.setEnabled(True)


    def T5_sub(self,pressed):
        if pressed:
            self.pushButton_6.setEnabled(False)
            self.sin_T5_data.emit([5,-self.joint_velocity])
            pass

        else:
            self.sin_T5_data.emit([5,0])
            self.pushButton_6.setEnabled(True)
        pass


    def T6_add(self,pressed):

        if pressed:
            self.pushButton_12.setEnabled(False)
            self.sin_T6_data.emit([6,self.joint_velocity])
            pass

        else:
            self.sin_T6_data.emit([6,0])
            self.pushButton_12.setEnabled(True)

    def T6_sub(self,pressed):

        if pressed:
            self.pushButton_11.setEnabled(False)
            self.sin_T6_data.emit([6,-self.joint_velocity])
            pass

        else:
            self.sin_T6_data.emit([6,0])
            self.pushButton_11.setEnabled(True)
        pass

    def I7_add(self,pressed):
        if pressed:
            self.pushButton_14.setEnabled(False)
            self.sin_I7_data.emit([7,self.joint_velocity])
            pass

        else:
            self.sin_I7_data.emit([7,0])
            self.pushButton_14.setEnabled(True)
        pass

    def I7_sub(self,pressed):
        if pressed:
            self.pushButton_13.setEnabled(False)
            self.sin_I7_data.emit([7,-self.joint_velocity])
            pass

        else:
            self.sin_I7_data.emit([7,0])
            self.pushButton_13.setEnabled(True)
        pass

    def return_last_ui(self):
        '''
        return last ui and sent real robot current data.
        :return:
        '''
        self.sin_return_last_ui.emit()

    def change_velocity(self,data):
        # '''
        # move the slider to change the joint_velocity.
        # :param data:
        # :return:
        # '''
        self.joint_velocity = data * 0.001  # rad
        # print self.joint_velocity
        pass

    def set_zero(self):
        self.sin_set_zero.emit()
        pass

    def return_zero(self):
        self.sin_return_zero.emit()
        pass

    def quick_stop(self):
        self.sin_quick_stop.emit()
        pass