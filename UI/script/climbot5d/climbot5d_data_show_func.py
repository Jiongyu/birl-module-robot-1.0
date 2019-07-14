#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
sys.path.append("./src/birl_module_robot/UI/scripts/climbot5d")
from math import fabs,degrees,radians
import time
from PyQt5.QtWidgets import QWidget,QDesktopWidget,QFileDialog,QMessageBox
from PyQt5.QtCore import pyqtSignal
from climbot5d_data_show import Ui_climbot5d_Data_show
from climbot5d_sent_offline_data import Climbot5d_interpolation
from climbot5d_data_show_monitor_func import Climbot5d_data_show_monitor_func
from climbot5d_variable import Climbot5d_Variable


class climbot5d_Data_show_func(QWidget,Ui_climbot5d_Data_show,Climbot5d_Variable):

    sin_return_last_ui = pyqtSignal()
    sin_joint_data = pyqtSignal(list)
    sin_actual_line = pyqtSignal(int)
    sin_jump_to_a_line = pyqtSignal(int)

    sin_G0_data = pyqtSignal(int)
    sin_G0_command = pyqtSignal(int)
    sin_G6_data = pyqtSignal(int)
    sin_G6_command = pyqtSignal(int)

    sin_offline_data = pyqtSignal(list)
    sin_vel = pyqtSignal(float)
    
    sin_quick_stop = pyqtSignal()

    sin_open_monitor = pyqtSignal()
    sin_data_monitor = pyqtSignal(list)

    sin_pause_data_show = pyqtSignal(bool)

    def __init__(self,parent=None):
        super(climbot5d_Data_show_func,self).__init__(parent)
        Climbot5d_Variable.__init__(self)
        self.setupUi(self)
        self.center()
        # self.pushButton_6.setEnabled(False)
        # self.pushButton_2.setEnabled(False)
        # self.pushButton_3.setEnabled(False)

        # self.pushButton_13.setEnabled(False)
        # self.pushButton_12.setEnabled(False)
        # self.pushButton_11.setEnabled(False)
        # self.pushButton_14.setEnabled(False)

        self.joint_value = []
        self.actual_joint_value = []
        self.actual_line = 0

        self.sin_G0_command.connect(self.G0_sent_torque)
        self.sin_G6_command.connect(self.G6_sent_torque)

        self.sent_command = Climbot5d_interpolation()
        self.sin_offline_data.connect(self.sent_command.receive_offline)
        self.sin_vel.connect(self.sent_command.receive_max_joint_vel)
        self.sent_command.sin_interpolation_data.connect(self.get_interpolation_data)
        
        self.velocity = 0.02

    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())




    # 载入文件
    def load_data(self):
        
        text = self.textEdit.toPlainText()
        if text:
            try:
                data = text.split('\n')
                joint_data = []
                for i in range(len(data)):
                    line = data[i].split(',')
                    joint_data.append(line)
                # print joint_data
                a = []
                self.joint_value = [[]] * len(joint_data)
                for i in range(len(joint_data)):
                    # print joint_data[i]
                    for j in range(len(joint_data[i])):
                        a.append(round(float(joint_data[i][j]),4))       
                    self.joint_value[i] = a
                    a = []
                del a
                del data
                del joint_data
                self.listWidget.addItem('<< 成功载入数据')
                # print self.joint_value
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
        else:
            self.sin_pause_data_show.emit(False)
        



    def start_data_show(self):
        self.sin_joint_data.emit([self.joint_value,True])

    def return_last_ui(self):
        self.sin_return_last_ui.emit()
        pass

    def change_velocity(self,data):
        # data (1~1000)
        self.velocity = data * 0.001
        self.listWidget.addItem("关节最大速度为：" + str(round(degrees(self.velocity),3)) + "deg/s.")
        #print self.velocity
        pass

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

            self.pushButton_6.setEnabled(True)
            self.pushButton_2.setEnabled(True)

    def G0_close(self, pressed):
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
            self.pushButton_6.setEnabled(False)
            self.pushButton_2.setEnabled(False)

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
            self.pushButton_6.setEnabled(True)
            self.pushButton_2.setEnabled(True)
            

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
            self.pushButton_6.setEnabled(False)
            self.pushButton_2.setEnabled(False)

    def G6_sent_torque(self, data):
        if data == 0:
            self.sin_G6_data.emit(0)
        else:
            self.G6_value = data
            self.sin_G6_data.emit(data)

    def sent_offline_data(self):
        
        self.sin_offline_data.emit(self.joint_value)
        self.sin_vel.emit(round(degrees(self.velocity),1))
        self.sent_command.start()
    
    def quit_thread(self):
        self.sent_command.quit()

    def get_interpolation_data(self,data):
        self.joint_value = data
        self.textEdit.clear()
        line = ""
        for i in range(len(self.joint_value)):

            line_1 = "pos:" + str(round(degrees(self.joint_value[i][0]),1)) + "," + str(round(degrees(self.joint_value[i][1]),1)) + "," + str(round(degrees(self.joint_value[i][2]),1)) + "," \
                        + str(round(degrees(self.joint_value[i][3]),1)) + "," + str(round(degrees(self.joint_value[i][4]),1)) + "\n"\
                     "vel:" + str(round(degrees(self.joint_value[i][5]),1)) + "," + str(round(degrees(self.joint_value[i][6]),1)) + "," + str(round(degrees(self.joint_value[i][7]),1)) + "," \
                        + str(round(degrees(self.joint_value[i][8]),1)) + "," + str(round(degrees(self.joint_value[i][9]),1)) + "\n\n"
            line += line_1
        self.textEdit.setText(line)
        self.listWidget.addItem(">>新插值数据.")
        
    def quick_stop(self):
        self.sin_quick_stop.emit()

    def data_monitor(self):
        self.monitor = Climbot5d_data_show_monitor_func()
        self.sin_data_monitor.connect(self.monitor.show_data)
        self.monitor.show()

    def sent_data_monitor(self,data):
        self.sin_data_monitor.emit(data)


    def get_path_planning_data(self):
        pass


    

