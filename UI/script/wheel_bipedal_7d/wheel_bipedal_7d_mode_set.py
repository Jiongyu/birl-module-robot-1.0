# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'wheel_bipedal_7d_mode_set.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_wheel_bipedal_7d_mode_set(object):
    def setupUi(self, wheel_bipedal_7d_mode_set):
        wheel_bipedal_7d_mode_set.setObjectName("wheel_bipedal_7d_mode_set")
        wheel_bipedal_7d_mode_set.resize(180, 278)
        self.form = wheel_bipedal_7d_mode_set
        self.pushButton = QtWidgets.QPushButton(wheel_bipedal_7d_mode_set)
        self.pushButton.setGeometry(QtCore.QRect(20, 80, 140, 50))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_3 = QtWidgets.QPushButton(wheel_bipedal_7d_mode_set)
        self.pushButton_3.setGeometry(QtCore.QRect(0, 0, 51, 25))
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_4 = QtWidgets.QPushButton(wheel_bipedal_7d_mode_set)
        self.pushButton_4.setGeometry(QtCore.QRect(20, 180, 140, 50))
        self.pushButton_4.setObjectName("pushButton_4")

        self.retranslateUi(wheel_bipedal_7d_mode_set)
        self.pushButton_3.clicked.connect(wheel_bipedal_7d_mode_set.return_last_ui)
        self.pushButton.clicked.connect(wheel_bipedal_7d_mode_set.joint_control)
        self.pushButton_4.clicked.connect(wheel_bipedal_7d_mode_set.data_show)
        QtCore.QMetaObject.connectSlotsByName(wheel_bipedal_7d_mode_set)

    def retranslateUi(self, wheel_bipedal_7d_mode_set):
        _translate = QtCore.QCoreApplication.translate
        wheel_bipedal_7d_mode_set.setWindowTitle(_translate("wheel_bipedal_7d_mode_set", "模式选择"))
        self.pushButton.setText(_translate("wheel_bipedal_7d_mode_set", "关节空间控制"))
        self.pushButton_3.setText(_translate("wheel_bipedal_7d_mode_set", "返回"))
        self.pushButton_4.setText(_translate("wheel_bipedal_7d_mode_set", "数据再现"))

