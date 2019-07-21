#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
sys.path.append("./src/birl_module_robot/UI/scripts/arm5d")
import time
from  math import fabs,degrees,radians
from PyQt5.QtWidgets import QWidget,QDesktopWidget
from PyQt5.QtCore import pyqtSignal
from arm5d_descartes_control import Ui_arm5d_descart_control
from arm5d_teach_record_func import Arm5d_teach_record_func

class Arm5d_descartes_control_func(QWidget,Ui_arm5d_descart_control):

    sin_X_data = pyqtSignal(list) 
    sin_Y_data = pyqtSignal(list)
    sin_Z_data = pyqtSignal(list)
    sin_RX_data = pyqtSignal(list)
    sin_RY_data =pyqtSignal(list)
    sin_RZ_data = pyqtSignal(list)

    sin_return_last_ui = pyqtSignal()
    sin_quick_stop = pyqtSignal()

    #示教使用
    sin_teach_data = pyqtSignal(list) 

    def __init__(self):
        super(Arm5d_descartes_control_func,self).__init__()
        self.setupUi(self)
        self.center()
        self.descartes_velocity = 0.002

    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def X_data(self,pressed):
        '''
        push button to add X displacement.
        :param pressed:
        :return:
        '''
        if pressed:
            data = self.lineEdit_14.text()
            if data:
                self.sin_X_data.emit([0.001*float(data),self.descartes_velocity])

    def Y_data(self,pressed):
        if pressed:
            data = self.lineEdit_16.text()
            if data:
                self.sin_Y_data.emit([0.001*float(data),self.descartes_velocity])

    def Z_data(self,pressed):
        if pressed:
            data = self.lineEdit_18.text()
            if data:
                self.sin_Z_data.emit([0.001*float(data),self.descartes_velocity])
            pass

    def RX_data(self,pressed):
        if pressed:
            data = self.lineEdit_20.text()
            if data:
                self.sin_RX_data.emit([float(data),self.descartes_velocity])

    def RY_data(self,pressed):
        if pressed:
            data = self.lineEdit_21.text()
            if data:
                self.sin_RY_data.emit([float(data),self.descartes_velocity])

    def RZ_data(self,pressed):
        if pressed:
            data = self.lineEdit_23.text()
            if data:
                self.sin_RZ_data.emit([float(data),self.descartes_velocity])

    def return_last_ui(self):
        '''
        return last ui and sent all current robot data.
        :return:
        '''
        self.sin_return_last_ui.emit()
        #del data
        pass

    def change_velocity(self,data):
        '''
        move the slider to change the descartes_velocity.
        :param data:
        :return:
        '''
        self.descartes_velocity = data * 0.0001  # rad
        self.listWidget.addItem(">>>速度:{}m/s".format(round(self.descartes_velocity,4)))
        # print self.descartes_velocity
        pass

    def quick_stop(self):
        self.sin_quick_stop.emit()

    def teach_record(self):
        self.__windows_teach_record = Arm5d_teach_record_func()
        self.sin_teach_data.connect(self.__windows_teach_record.get_joint_data)
        self.__windows_teach_record.show()
        pass

    def receive_teach_data(self,data):
        self.sin_teach_data.emit(data)
        pass