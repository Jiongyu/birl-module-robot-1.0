# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'wheel_bipedal_7d_data_show.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_wheel_bipdeal_7d_data_show(object):
    def setupUi(self, wheel_bipdeal_7d_data_show):
        wheel_bipdeal_7d_data_show.setObjectName("wheel_bipdeal_7d_data_show")
        wheel_bipdeal_7d_data_show.resize(711, 600)
        self.pushButton_5 = QtWidgets.QPushButton(wheel_bipdeal_7d_data_show)
        self.pushButton_5.setGeometry(QtCore.QRect(0, 0, 50, 30))
        self.pushButton_5.setObjectName("pushButton_5")
        self.label_2 = QtWidgets.QLabel(wheel_bipdeal_7d_data_show)
        self.label_2.setGeometry(QtCore.QRect(370, 10, 67, 17))
        self.label_2.setObjectName("label_2")
        self.pushButton_7 = QtWidgets.QPushButton(wheel_bipdeal_7d_data_show)
        self.pushButton_7.setGeometry(QtCore.QRect(80, 0, 70, 30))
        self.pushButton_7.setObjectName("pushButton_7")
        self.listWidget = QtWidgets.QListWidget(wheel_bipdeal_7d_data_show)
        self.listWidget.setGeometry(QtCore.QRect(370, 40, 321, 340))
        self.listWidget.setObjectName("listWidget")
        self.pushButton_2 = QtWidgets.QPushButton(wheel_bipdeal_7d_data_show)
        self.pushButton_2.setGeometry(QtCore.QRect(400, 400, 70, 60))
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_6 = QtWidgets.QPushButton(wheel_bipdeal_7d_data_show)
        self.pushButton_6.setGeometry(QtCore.QRect(400, 510, 70, 60))
        self.pushButton_6.setObjectName("pushButton_6")
        self.pushButton_6.setCheckable(True)
        self.pushButton_4 = QtWidgets.QPushButton(wheel_bipdeal_7d_data_show)
        self.pushButton_4.setGeometry(QtCore.QRect(570, 470, 100, 100))
        self.pushButton_4.setStyleSheet("background-color: rgb(239, 41, 41);")
        self.pushButton_4.setObjectName("pushButton_4")
        self.horizontalSlider = QtWidgets.QSlider(wheel_bipdeal_7d_data_show)
        self.horizontalSlider.setGeometry(QtCore.QRect(570, 410, 121, 20))
        self.horizontalSlider.setMaximum(1000)
        self.horizontalSlider.setSingleStep(100)
        self.horizontalSlider.setProperty("value", 50)
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setObjectName("horizontalSlider")
        self.label = QtWidgets.QLabel(wheel_bipdeal_7d_data_show)
        self.label.setGeometry(QtCore.QRect(610, 430, 40, 20))
        self.label.setObjectName("label")
        self.pushButton_8 = QtWidgets.QPushButton(wheel_bipdeal_7d_data_show)
        self.pushButton_8.setGeometry(QtCore.QRect(260, 560, 70, 30))
        self.pushButton_8.setObjectName("pushButton_8")
        self.textEdit = QtWidgets.QTextEdit(wheel_bipdeal_7d_data_show)
        self.textEdit.setGeometry(QtCore.QRect(13, 39, 341, 511))
        self.textEdit.setObjectName("textEdit")

        self.retranslateUi(wheel_bipdeal_7d_data_show)
        self.pushButton_8.clicked.connect(wheel_bipdeal_7d_data_show.load_data)
        self.pushButton_2.clicked.connect(wheel_bipdeal_7d_data_show.start_data_show)
        self.pushButton_6.clicked['bool'].connect(wheel_bipdeal_7d_data_show.pause_data_show)
        self.pushButton_7.clicked.connect(wheel_bipdeal_7d_data_show.data_monitor)
        self.pushButton_4.clicked.connect(wheel_bipdeal_7d_data_show.quick_stop)
        self.horizontalSlider.valueChanged['int'].connect(wheel_bipdeal_7d_data_show.change_velocity)
        self.pushButton_5.clicked.connect(wheel_bipdeal_7d_data_show.return_last_ui)
        QtCore.QMetaObject.connectSlotsByName(wheel_bipdeal_7d_data_show)

    def retranslateUi(self, wheel_bipdeal_7d_data_show):
        _translate = QtCore.QCoreApplication.translate
        wheel_bipdeal_7d_data_show.setWindowTitle(_translate("wheel_bipdeal_7d_data_show", "数据再现"))
        self.pushButton_5.setText(_translate("wheel_bipdeal_7d_data_show", "返回"))
        self.label_2.setText(_translate("wheel_bipdeal_7d_data_show", "执行过程"))
        self.pushButton_7.setText(_translate("wheel_bipdeal_7d_data_show", "数据监控"))
        self.pushButton_2.setText(_translate("wheel_bipdeal_7d_data_show", "运行"))
        self.pushButton_6.setText(_translate("wheel_bipdeal_7d_data_show", "暂停"))
        self.pushButton_4.setText(_translate("wheel_bipdeal_7d_data_show", "急停"))
        self.label.setText(_translate("wheel_bipdeal_7d_data_show", "速度"))
        self.pushButton_8.setText(_translate("wheel_bipdeal_7d_data_show", "载入数据"))

