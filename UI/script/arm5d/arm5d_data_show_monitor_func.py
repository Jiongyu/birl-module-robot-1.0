#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
sys.path.append("./src/birl_module_robot/UI/scripts/arm5d")
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget,QDesktopWidget,QFileDialog
from arm5d_data_show_monitor import Ui_arm5d_data_show_monitor

from math import degrees

class Arm5d_data_show_monitor_func(QWidget,Ui_arm5d_data_show_monitor):
    
    sin_open_monitor = pyqtSignal()

    def __init__(self):
        super(Arm5d_data_show_monitor_func,self).__init__()
        self.setupUi(self)

    def show_data(self,data):
        # I1,T2,T3,i4,t5
        self.lineEdit_2.setText("{0}".format(round(degrees(data[1]),3)))  
        self.lineEdit_3.setText("{0}".format(round(degrees(data[2]),3)))
        self.lineEdit_5.setText("{0}".format(round(degrees(data[3]),3)))
        self.lineEdit_4.setText("{0}".format(round(degrees(data[4]),3)))
        self.lineEdit_6.setText("{0}".format(round(degrees(data[5]),3)))
        # X,Y,Z,RX,RY,RZ
        self.lineEdit_11.setText("{0}".format(1000*round(data[7],3)))
        self.lineEdit_8.setText("{0}".format(1000*round(data[8],3)))
        self.lineEdit_7.setText("{0}".format(1000*round(data[9],3)))
        self.lineEdit_9.setText("{0}".format(round(degrees(data[10]),3)))
        self.lineEdit_10.setText("{0}".format(round(degrees(data[11]),3)))
        self.lineEdit_17.setText("{0}".format(round(degrees(data[11]),3)))

        pass
    
    