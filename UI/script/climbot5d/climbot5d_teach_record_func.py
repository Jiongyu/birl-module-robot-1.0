#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
sys.path.append("./src/birl_module_robot/UI/scripts/climbot5d")
from os.path import exists

from rospkg import RosPack
rp = RosPack()

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget,QDesktopWidget,QFileDialog,QApplication
from climbot5d_teach_record import Ui_climbot5d_teach_record

class Climbot5d_teach_record_func(QWidget,Ui_climbot5d_teach_record):

    def __init__(self):
        super(Climbot5d_teach_record_func,self).__init__()
        self.setupUi(self)

        self.__teach_data_index = 0
        self.__file_path = rp.get_path('ui') + "/teach_data/teach_data_{0}.txt".format(self.__teach_data_index)
        self.__I1_pos,self.__T2_pos,self.__T3_pos,self.__T4_pos,self.__I5_pos = [0,0,0,0,0]
        self.__click_line = -1
        self.__clear = False

    def take_record(self):
        if self.__clear:
            self.listWidget.clear()
            self.__clear = False
        self.listWidget.addItem("{0},{1},{2},{3},{4}".format(self.__I1_pos,self.__T2_pos,self.__T3_pos,self.__T4_pos,self.__I5_pos))
        pass

    def delete_data(self):
        if self.__click_line != -1:
            self.listWidget.takeItem(self.__click_line)
        pass

    def save_teach_file(self):

        while True:
            if not exists(self.__file_path):
                with open (self.__file_path,'w+') as f:
                    for i in range(self.listWidget.count()):
                        f.write("{0}\n".format(self.listWidget.item(i)))
                        pass
                f.close()
                self.listWidget.addItem("save file: teach_data_{0}.txt".format(self.__teach_data_index))
                self.__clear = True
                break
            else:
                self.__teach_data_index += 1
                self.__file_path = rp.get_path('ui') + "/teach_data/teach_data_{0}.txt".format(self.__teach_data_index)
            QApplication.processEvents()
        pass

    def get_joint_data(self,data):
        self.__I1_pos,self.__T2_pos,self.__T3_pos,self.__T4_pos,self.__I5_pos = data 

    def get_click_line(self,line):
        self.__click_line = line.row()
        pass