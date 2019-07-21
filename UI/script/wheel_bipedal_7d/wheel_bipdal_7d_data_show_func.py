#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
sys.path.append("./src/birl_module_robot/UI/scripts/wheel_bipedal_7d")
import string
from math import fabs,degrees,radians
import time
from PyQt5.QtWidgets import QWidget,QDesktopWidget,QFileDialog,QMessageBox
from PyQt5.QtCore import pyqtSignal
from wheel_bipedal_7d_data_show import Ui_wheel_bipdeal_7d_data_show

from wheel_bipedal7d_data_monitor_func import Wheel_bipedal7d_data_monitorfunc

class Wheel_bipedal_7d_data_show_func(QWidget,Ui_wheel_bipdeal_7d_data_show):

    sin_return_last_ui = pyqtSignal()
    sin_joint_data = pyqtSignal(list)
    sin_actual_line = pyqtSignal(int)
    sin_jump_to_a_line = pyqtSignal(int)

    sin_vel = pyqtSignal(float)
    
    sin_quick_stop = pyqtSignal()

    sin_open_monitor = pyqtSignal()
    sin_data_monitor = pyqtSignal(list)

    sin_pause_data_show = pyqtSignal(bool)

    def __init__(self,parent=None):
        super(Wheel_bipedal_7d_data_show_func,self).__init__(parent)
        self.setupUi(self)
        self.center()

        self.joint_value = []
        self.actual_joint_value = []
        self.actual_line = 0
        
        self.velocity = 0.02
        self.joint_velocity_ = []


        self.monitor = Wheel_bipedal7d_data_monitorfunc()
        self.sin_data_monitor.connect(self.monitor.show_data)

    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    # 载入文件
    def load_data(self):
        self.joint_value = []
        self.joint_velocity_ = []
        text = self.textEdit.toPlainText()
        if text:
            try:
                data = text.split('\n')

                for i in range(len(data)):
                    temp1 = data[i].replace(',','')    #删除逗号
                    temp2 = temp1.replace(';','')   #删除分号
                    data[i] = temp2.replace('=',' ')  #用空格替代等号
                    # print data[i]
                    if data[i].startswith('P'):      
                        s = data[i].split()    #字符串按照空格分开，split()默认以空格分开 
                        s[1] = round(radians(string.atof(s[1])),3)    #字符串转化为浮点数
                        s[2] = round(radians(string.atof(s[2])),3)    #字符串转浮点数
                        s[3] = round(radians(string.atof(s[3])),3)
                        s[4] = round(radians(string.atof(s[4])),3)
                        s[5] = round(radians(string.atof(s[5])),3)
                        s[6] = round(radians(string.atof(s[6])),3)
                        s[7] = round(radians(string.atof(s[7])),3)
                        self.joint_value.append(s[1:8])
                    else:
                        if data[i].startswith('V'):
                            v = data[i].split()
                            # print v
                            # v[1] =  round(radians(string.atof(v[1])),3)    #字符串转化为浮点数
                            # v[2] =  round(radians(string.atof(v[2])),3)    #字符串转浮点数
                            # v[3] =  round(radians(string.atof(v[3])),3)
                            # v[4] =  round(radians(string.atof(v[4])),3)
                            # v[5] =  round(radians(string.atof(v[5])),3)
                            # v[6] =  round(radians(string.atof(v[6])),3)
                            # v[7] =  round(radians(string.atof(v[7])),3)

                            v[1] =  abs( round(radians(string.atof(v[1])),3) )    #字符串转化为浮点数
                            v[2] =  abs( round(radians(string.atof(v[2])),3) )    #字符串转浮点数
                            v[3] =  abs( round(radians(string.atof(v[3])),3) )
                            v[4] =  abs( round(radians(string.atof(v[4])),3) )
                            v[5] =  abs( round(radians(string.atof(v[5])),3) )
                            v[6] =  abs( round(radians(string.atof(v[6])),3) )
                            v[7] =  abs( round(radians(string.atof(v[7])),3) )


                            self.joint_velocity_.append(v[1:8])
                            # print v[1:8]

                self.listWidget.addItem('<< 成功载入数据')
                # print self.joint_value
                # print self.joint_velocity_  
            except:
                self.__box = QMessageBox(QMessageBox.Warning, "错误", "文件载入错误(请确认数据格式,文件格式).")
                self.__box.addButton(self.tr("确定"), QMessageBox.YesRole)
                self.__box.exec_()

        else:
            self.listWidget.addItem('<< 空数据')
            pass
    
    # 暂停
    def pause_data_show(self,pressed):
        if pressed:
            self.sin_pause_data_show.emit(True)
            self.listWidget.addItem("<<暂停运行")
        else:
            self.sin_pause_data_show.emit(False)
            self.listWidget.addItem("<<继续运行")


    def start_data_show(self):
        self.sin_joint_data.emit([self.joint_value,True,self.joint_velocity_])
        self.listWidget.addItem("<<开始运行")

    def return_last_ui(self):
        self.sin_return_last_ui.emit()
        pass

    def change_velocity(self,data):
        # data (1~1000)
        self.velocity = data * 0.001
        self.listWidget.addItem("关节最大速度为：" + str(round(degrees(self.velocity),3)) + "deg/s.")
        #print self.velocity
        pass

    def sent_offline_data(self):
        
        self.sin_offline_data.emit(self.joint_value)
        self.sin_vel.emit(round(degrees(self.velocity),1))
        self.sent_command.start()

    def quick_stop(self):
        self.sin_quick_stop.emit()
        self.listWidget.addItem("<<急停")

    def data_monitor(self):
        self.monitor.show()
        pass

    def sent_data_monitor(self,data):
        self.sin_data_monitor.emit(data)
