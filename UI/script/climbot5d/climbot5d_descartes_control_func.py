#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
sys.path.append("./src/birl_module_robot/UI/scripts/climbot5d")
import time
from  math import fabs,degrees,radians
from PyQt5.QtWidgets import QWidget,QDesktopWidget
from PyQt5.QtCore import pyqtSignal
from climbot5d_descartes_control import Ui_Climbot5d_Descartes_control
from climbot5d_teach_record_func import Climbot5d_teach_record_func
from climbot5d_variable import Climbot5d_Variable

class Climbot5d_descartes_control_func(QWidget, Ui_Climbot5d_Descartes_control,Climbot5d_Variable):


    sin_X_data = pyqtSignal(list) 
    sin_Y_data = pyqtSignal(list)
    sin_Z_data = pyqtSignal(list)
    sin_RX_data = pyqtSignal(list)
    sin_RY_data =pyqtSignal(list)
    sin_RZ_data = pyqtSignal(list)

    sin_G0_data = pyqtSignal(int)
    sin_G0_command = pyqtSignal(int)

    sin_G6_data = pyqtSignal(int)
    sin_G6_command = pyqtSignal(int)

    sin_return_last_ui = pyqtSignal()
    sin_quick_stop = pyqtSignal()

    #示教使用
    sin_teach_data = pyqtSignal(list) 

    def __init__(self):
        super(Climbot5d_descartes_control_func,self).__init__()
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
            self.sin_RZ_data.emit([float(data),self.descartes_velocity])

    def G0_open(self, pressed):
        '''
        push button to open gripper 0.
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
            self.pushButton_3.setEnabled(True)
            self.pushButton_4.setEnabled(True)
            self.pushButton_5.setEnabled(True)
            self.pushButton_6.setEnabled(True)
            self.pushButton_15.setEnabled(True)

    def G0_close(self, pressed):

        if pressed:
            self.sin_G0_command.emit(self.close_gripper)
            self.listWidget.addItem(">>夹持器0关闭．")
            pass

        else:
            self.sin_G0_command.emit(self.zero_gripper)
            self.pushButton_14.setEnabled(True)
            self.pushButton_13.setEnabled(True)
            self.pushButton_11.setEnabled(True)
            self.pushButton_1.setEnabled(False)
            self.pushButton_3.setEnabled(False)
            self.pushButton_4.setEnabled(False)
            self.pushButton_5.setEnabled(False)
            self.pushButton_6.setEnabled(False)
            self.pushButton_15.setEnabled(False)
        pass

    def G0_sent_torque(self, data):
        if data == 0:
            self.sin_G0_data.emit(data)
        else:
            self.G0_value = data
            self.sin_G0_data.emit(data)

    def G6_open(self, pressed):

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
            self.pushButton_3.setEnabled(True)
            self.pushButton_4.setEnabled(True)
            self.pushButton_5.setEnabled(True)
            self.pushButton_6.setEnabled(True)
            self.pushButton_15.setEnabled(True)


    def G6_close(self, pressed):

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
            self.pushButton_3.setEnabled(False)
            self.pushButton_4.setEnabled(False)
            self.pushButton_5.setEnabled(False)
            self.pushButton_6.setEnabled(False)
            self.pushButton_15.setEnabled(False)
        pass

    def G6_sent_torque(self, data):
        if data == 0:
            self.sin_G6_data.emit(0)
        else:
            self.G6_value = data
            self.sin_G6_data.emit(data)

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
        self.__windows_teach_record = Climbot5d_teach_record_func()
        self.sin_teach_data.connect(self.__windows_teach_record.get_joint_data)
        self.__windows_teach_record.show()
        pass

    def receive_teach_data(self,data):
        self.sin_teach_data.emit(data)
        pass
        
