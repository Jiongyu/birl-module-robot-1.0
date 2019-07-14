# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'robot_choice.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(245, 400)
        self.form = MainWindow
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(20, 20, 200, 50))
        self.pushButton.setObjectName("pushButton")
        self.checkBox = QtWidgets.QCheckBox(self.centralwidget)
        self.checkBox.setGeometry(QtCore.QRect(165, 320, 60, 23))
        self.checkBox.setObjectName("checkBox")
        # self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget)
        # self.pushButton_2.setGeometry(QtCore.QRect(20, 90, 200, 50))
        # self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_3.setGeometry(QtCore.QRect(20, 170, 200, 50))
        self.pushButton_3.setObjectName("pushButton_3")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 240, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.pushButton.clicked.connect(MainWindow.climbot5d)
        self.checkBox.clicked['bool'].connect(MainWindow.if_simulation)
        # self.pushButton_2.clicked.connect(MainWindow.robot_arm5d)
        self.pushButton_3.clicked.connect(MainWindow.wheel_bipedal7d)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "机器人选择"))
        self.pushButton.setText(_translate("MainWindow", "5-DOF双手爪攀爬机器人"))
        self.checkBox.setText(_translate("MainWindow", "仿真"))
        # self.pushButton_2.setText(_translate("MainWindow", "5-DOF机器手"))
        self.pushButton_3.setText(_translate("MainWindow", "7-DOF轮足式机器人"))

