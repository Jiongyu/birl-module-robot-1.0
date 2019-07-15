#!/usr/bin/env python
# -*- coding: utf-8 -*-
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget,QDesktopWidget,QFileDialog
from wheel_bipedal7d_data_monitor import Ui_wheel_bipedal7d_data_monitor

from math import degrees

class Wheel_bipedal7d_data_monitorfunc(QWidget,Ui_wheel_bipedal7d_data_monitor):
    
    sin_open_monitor = pyqtSignal()

    def __init__(self):
        super(Wheel_bipedal7d_data_monitorfunc,self).__init__()
        self.setupUi(self)

    def show_data(self,data):
        self.lineEdit_2.setText("{0}".format(round(degrees(data[0]),3)))  
        self.lineEdit_3.setText("{0}".format(round(degrees(data[1]),3)))
        self.lineEdit_5.setText("{0}".format(round(degrees(data[2]),3)))
        self.lineEdit_4.setText("{0}".format(round(degrees(data[3]),3)))
        self.lineEdit_6.setText("{0}".format(round(degrees(data[4]),3)))
        self.lineEdit_7.setText("{0}".format(round(degrees(data[5]),3)))
        self.lineEdit_8.setText("{0}".format(round(degrees(data[6]),3)))


        pass
    
    