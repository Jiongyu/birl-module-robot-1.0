# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'arm5d_mode_set.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_arm5d_mode_set(object):
    def setupUi(self, arm5d_mode_set):
        arm5d_mode_set.setObjectName("arm5d_mode_set")
        arm5d_mode_set.resize(239, 306)
        self.form = arm5d_mode_set
        self.pushButton = QtWidgets.QPushButton(arm5d_mode_set)
        self.pushButton.setGeometry(QtCore.QRect(50, 50, 140, 50))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QtWidgets.QPushButton(arm5d_mode_set)
        self.pushButton_2.setGeometry(QtCore.QRect(50, 130, 140, 50))
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_3 = QtWidgets.QPushButton(arm5d_mode_set)
        self.pushButton_3.setGeometry(QtCore.QRect(0, 0, 51, 25))
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_4 = QtWidgets.QPushButton(arm5d_mode_set)
        self.pushButton_4.setGeometry(QtCore.QRect(50, 220, 140, 50))
        self.pushButton_4.setObjectName("pushButton_4")

        self.retranslateUi(arm5d_mode_set)
        self.pushButton.clicked.connect(arm5d_mode_set.joint_control)
        self.pushButton_2.clicked.connect(arm5d_mode_set.decartes_control)
        self.pushButton_3.clicked.connect(arm5d_mode_set.return_last_ui)
        self.pushButton_4.clicked.connect(arm5d_mode_set.data_show)
        QtCore.QMetaObject.connectSlotsByName(arm5d_mode_set)

    def retranslateUi(self, arm5d_mode_set):
        _translate = QtCore.QCoreApplication.translate
        arm5d_mode_set.setWindowTitle(_translate("arm5d_mode_set", "模式选择"))
        self.pushButton.setText(_translate("arm5d_mode_set", "关节空间控制"))
        self.pushButton_2.setText(_translate("arm5d_mode_set", "笛卡尔空间控制"))
        self.pushButton_3.setText(_translate("arm5d_mode_set", "返回"))
        self.pushButton_4.setText(_translate("arm5d_mode_set", "数据再现"))

