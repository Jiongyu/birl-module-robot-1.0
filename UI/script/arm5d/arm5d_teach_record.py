# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'arm5d_teach_record.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_arm5d_teach_record(object):
    def setupUi(self, arm5d_teach_record):
        arm5d_teach_record.setObjectName("arm5d_teach_record")
        arm5d_teach_record.resize(381, 331)
        self.pushButton_2 = QtWidgets.QPushButton(arm5d_teach_record)
        self.pushButton_2.setGeometry(QtCore.QRect(280, 20, 89, 50))
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_3 = QtWidgets.QPushButton(arm5d_teach_record)
        self.pushButton_3.setGeometry(QtCore.QRect(280, 260, 89, 50))
        self.pushButton_3.setObjectName("pushButton_3")
        self.listWidget = QtWidgets.QListWidget(arm5d_teach_record)
        self.listWidget.setGeometry(QtCore.QRect(10, 10, 256, 311))
        self.listWidget.setObjectName("listWidget")
        self.pushButton_4 = QtWidgets.QPushButton(arm5d_teach_record)
        self.pushButton_4.setGeometry(QtCore.QRect(280, 140, 89, 50))
        self.pushButton_4.setObjectName("pushButton_4")

        self.retranslateUi(arm5d_teach_record)
        self.pushButton_4.clicked.connect(arm5d_teach_record.delete_data)
        self.listWidget.clicked.connect(arm5d_teach_record.get_click_line)
        self.pushButton_2.clicked.connect(arm5d_teach_record.take_record)
        self.pushButton_3.clicked.connect(arm5d_teach_record.save_teach_file)
        QtCore.QMetaObject.connectSlotsByName(arm5d_teach_record)

    def retranslateUi(self, arm5d_teach_record):
        _translate = QtCore.QCoreApplication.translate
        arm5d_teach_record.setWindowTitle(_translate("arm5d_teach_record", "示教记录"))
        self.pushButton_2.setText(_translate("arm5d_teach_record", "记录"))
        self.pushButton_3.setText(_translate("arm5d_teach_record", "保存"))
        self.pushButton_4.setText(_translate("arm5d_teach_record", "删除"))

