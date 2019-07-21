# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'arm5d_data_show.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_arm5d_data_show(object):
    def setupUi(self, arm5d_data_show):
        arm5d_data_show.setObjectName("arm5d_data_show")
        arm5d_data_show.resize(734, 574)
        self.pushButton_6 = QtWidgets.QPushButton(arm5d_data_show)
        self.pushButton_6.setGeometry(QtCore.QRect(480, 490, 70, 60))
        self.pushButton_6.setObjectName("pushButton_6")
        self.pushButton_4 = QtWidgets.QPushButton(arm5d_data_show)
        self.pushButton_4.setGeometry(QtCore.QRect(610, 450, 100, 100))
        self.pushButton_4.setStyleSheet("background-color: rgb(239, 41, 41);")
        self.pushButton_4.setObjectName("pushButton_4")
        self.pushButton_5 = QtWidgets.QPushButton(arm5d_data_show)
        self.pushButton_5.setGeometry(QtCore.QRect(-3, 1, 50, 30))
        self.pushButton_5.setObjectName("pushButton_5")
        self.horizontalSlider = QtWidgets.QSlider(arm5d_data_show)
        self.horizontalSlider.setGeometry(QtCore.QRect(400, 410, 121, 20))
        self.horizontalSlider.setMaximum(100)
        self.horizontalSlider.setSingleStep(10)
        self.horizontalSlider.setProperty("value", 20)
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setObjectName("horizontalSlider")
        self.listWidget = QtWidgets.QListWidget(arm5d_data_show)
        self.listWidget.setGeometry(QtCore.QRect(367, 41, 350, 340))
        self.listWidget.setObjectName("listWidget")
        self.pushButton_2 = QtWidgets.QPushButton(arm5d_data_show)
        self.pushButton_2.setGeometry(QtCore.QRect(370, 490, 70, 60))
        self.pushButton_2.setObjectName("pushButton_2")
        self.label_2 = QtWidgets.QLabel(arm5d_data_show)
        self.label_2.setGeometry(QtCore.QRect(367, 11, 67, 17))
        self.label_2.setObjectName("label_2")
        self.pushButton_7 = QtWidgets.QPushButton(arm5d_data_show)
        self.pushButton_7.setGeometry(QtCore.QRect(77, 1, 70, 30))
        self.pushButton_7.setObjectName("pushButton_7")
        self.label = QtWidgets.QLabel(arm5d_data_show)
        self.label.setGeometry(QtCore.QRect(440, 430, 40, 20))
        self.label.setObjectName("label")
        self.pushButton_8 = QtWidgets.QPushButton(arm5d_data_show)
        self.pushButton_8.setGeometry(QtCore.QRect(280, 520, 70, 30))
        self.pushButton_8.setObjectName("pushButton_8")
        self.textEdit = QtWidgets.QTextEdit(arm5d_data_show)
        self.textEdit.setGeometry(QtCore.QRect(10, 40, 341, 461))
        self.textEdit.setObjectName("textEdit")

        self.retranslateUi(arm5d_data_show)
        self.pushButton_8.clicked.connect(arm5d_data_show.load_data)
        self.pushButton_2.clicked.connect(arm5d_data_show.start_data_show)
        self.pushButton_6.clicked.connect(arm5d_data_show.pause_data_show)
        self.pushButton_4.clicked.connect(arm5d_data_show.quick_stop)
        self.pushButton_7.clicked.connect(arm5d_data_show.data_monitor)
        self.horizontalSlider.valueChanged['int'].connect(arm5d_data_show.change_velocity)
        self.pushButton_5.clicked.connect(arm5d_data_show.return_last_ui)
        QtCore.QMetaObject.connectSlotsByName(arm5d_data_show)

    def retranslateUi(self, arm5d_data_show):
        _translate = QtCore.QCoreApplication.translate
        arm5d_data_show.setWindowTitle(_translate("arm5d_data_show", "离线示教"))
        self.pushButton_6.setText(_translate("arm5d_data_show", "暂停"))
        self.pushButton_4.setText(_translate("arm5d_data_show", "急停"))
        self.pushButton_5.setText(_translate("arm5d_data_show", "返回"))
        self.pushButton_2.setText(_translate("arm5d_data_show", "运行"))
        self.label_2.setText(_translate("arm5d_data_show", "执行过程"))
        self.pushButton_7.setText(_translate("arm5d_data_show", "数据监控"))
        self.label.setText(_translate("arm5d_data_show", "速度"))
        self.pushButton_8.setText(_translate("arm5d_data_show", "载入数据"))

