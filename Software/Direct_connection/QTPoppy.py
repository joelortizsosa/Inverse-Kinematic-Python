# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'C:\Users\aymar\Documents\Poppy\QTPoppy.ui'
#
# Created: Tue Aug 11 11:06:29 2015
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_Control_Poppy_Right_Arm(object):
    def setupUi(self, Control_Poppy_Right_Arm):
        Control_Poppy_Right_Arm.setObjectName(_fromUtf8("Control_Poppy_Right_Arm"))
        Control_Poppy_Right_Arm.resize(776, 641)
        Control_Poppy_Right_Arm.setBaseSize(QtCore.QSize(10, 0))
        self.centralwidget = QtGui.QWidget(Control_Poppy_Right_Arm)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.verticalLayoutWidget = QtGui.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 10, 751, 591))
        self.verticalLayoutWidget.setObjectName(_fromUtf8("verticalLayoutWidget"))
        self.verticalLayout = QtGui.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setMargin(0)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.X = QtGui.QLabel(self.verticalLayoutWidget)
        self.X.setObjectName(_fromUtf8("X"))
        self.horizontalLayout.addWidget(self.X)
        self.Slider_X = QtGui.QSlider(self.verticalLayoutWidget)
        self.Slider_X.setMaximumSize(QtCore.QSize(600, 16777215))
        self.Slider_X.setMinimum(-360)
        self.Slider_X.setMaximum(360)
        self.Slider_X.setProperty("value", 140)
        self.Slider_X.setOrientation(QtCore.Qt.Horizontal)
        self.Slider_X.setObjectName(_fromUtf8("Slider_X"))
        self.horizontalLayout.addWidget(self.Slider_X)
        self.X_value = QtGui.QLabel(self.verticalLayoutWidget)
        self.X_value.setObjectName(_fromUtf8("X_value"))
        self.horizontalLayout.addWidget(self.X_value)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_8 = QtGui.QHBoxLayout()
        self.horizontalLayout_8.setObjectName(_fromUtf8("horizontalLayout_8"))
        self.Y = QtGui.QLabel(self.verticalLayoutWidget)
        self.Y.setObjectName(_fromUtf8("Y"))
        self.horizontalLayout_8.addWidget(self.Y)
        self.Slider_Y = QtGui.QSlider(self.verticalLayoutWidget)
        self.Slider_Y.setMaximumSize(QtCore.QSize(600, 16777215))
        self.Slider_Y.setMaximum(360)
        self.Slider_Y.setProperty("value", 120)
        self.Slider_Y.setOrientation(QtCore.Qt.Horizontal)
        self.Slider_Y.setObjectName(_fromUtf8("Slider_Y"))
        self.horizontalLayout_8.addWidget(self.Slider_Y)
        self.Y_value = QtGui.QLabel(self.verticalLayoutWidget)
        self.Y_value.setObjectName(_fromUtf8("Y_value"))
        self.horizontalLayout_8.addWidget(self.Y_value)
        self.verticalLayout.addLayout(self.horizontalLayout_8)
        self.horizontalLayout_9 = QtGui.QHBoxLayout()
        self.horizontalLayout_9.setObjectName(_fromUtf8("horizontalLayout_9"))
        self.Z = QtGui.QLabel(self.verticalLayoutWidget)
        self.Z.setObjectName(_fromUtf8("Z"))
        self.horizontalLayout_9.addWidget(self.Z)
        self.Slider_Z = QtGui.QSlider(self.verticalLayoutWidget)
        self.Slider_Z.setMaximumSize(QtCore.QSize(600, 16777215))
        self.Slider_Z.setMinimum(-160)
        self.Slider_Z.setMaximum(360)
        self.Slider_Z.setProperty("value", 140)
        self.Slider_Z.setOrientation(QtCore.Qt.Horizontal)
        self.Slider_Z.setObjectName(_fromUtf8("Slider_Z"))
        self.horizontalLayout_9.addWidget(self.Slider_Z)
        self.Z_value = QtGui.QLabel(self.verticalLayoutWidget)
        self.Z_value.setObjectName(_fromUtf8("Z_value"))
        self.horizontalLayout_9.addWidget(self.Z_value)
        self.verticalLayout.addLayout(self.horizontalLayout_9)
        self.horizontalLayout_10 = QtGui.QHBoxLayout()
        self.horizontalLayout_10.setObjectName(_fromUtf8("horizontalLayout_10"))
        self.Wrist = QtGui.QLabel(self.verticalLayoutWidget)
        self.Wrist.setObjectName(_fromUtf8("Wrist"))
        self.horizontalLayout_10.addWidget(self.Wrist)
        self.Slider_wrist = QtGui.QSlider(self.verticalLayoutWidget)
        self.Slider_wrist.setMaximumSize(QtCore.QSize(600, 16777215))
        self.Slider_wrist.setOrientation(QtCore.Qt.Horizontal)
        self.Slider_wrist.setObjectName(_fromUtf8("Slider_wrist"))
        self.horizontalLayout_10.addWidget(self.Slider_wrist)
        self.Wrist_value = QtGui.QLabel(self.verticalLayoutWidget)
        self.Wrist_value.setObjectName(_fromUtf8("Wrist_value"))
        self.horizontalLayout_10.addWidget(self.Wrist_value)
        self.verticalLayout.addLayout(self.horizontalLayout_10)
        self.gridLayout = QtGui.QGridLayout()
        self.gridLayout.setContentsMargins(20, 50, 20, 50)
        self.gridLayout.setHorizontalSpacing(60)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.verticalLayout_4 = QtGui.QVBoxLayout()
        self.verticalLayout_4.setSizeConstraint(QtGui.QLayout.SetFixedSize)
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setSizeConstraint(QtGui.QLayout.SetFixedSize)
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.label_shoulderY = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_shoulderY.setMaximumSize(QtCore.QSize(160, 16777215))
        self.label_shoulderY.setObjectName(_fromUtf8("label_shoulderY"))
        self.horizontalLayout_2.addWidget(self.label_shoulderY)
        self.Shoulder_Y_value = QtGui.QLabel(self.verticalLayoutWidget)
        self.Shoulder_Y_value.setMaximumSize(QtCore.QSize(60, 16777215))
        self.Shoulder_Y_value.setObjectName(_fromUtf8("Shoulder_Y_value"))
        self.horizontalLayout_2.addWidget(self.Shoulder_Y_value)
        self.verticalLayout_4.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setSizeConstraint(QtGui.QLayout.SetFixedSize)
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.label_shoulderX = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_shoulderX.setMaximumSize(QtCore.QSize(160, 16777215))
        self.label_shoulderX.setObjectName(_fromUtf8("label_shoulderX"))
        self.horizontalLayout_3.addWidget(self.label_shoulderX)
        self.Shoulder_X_value = QtGui.QLabel(self.verticalLayoutWidget)
        self.Shoulder_X_value.setMaximumSize(QtCore.QSize(60, 16777215))
        self.Shoulder_X_value.setObjectName(_fromUtf8("Shoulder_X_value"))
        self.horizontalLayout_3.addWidget(self.Shoulder_X_value)
        self.verticalLayout_4.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setSizeConstraint(QtGui.QLayout.SetFixedSize)
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        self.label_elbowY = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_elbowY.setMaximumSize(QtCore.QSize(160, 16777215))
        self.label_elbowY.setObjectName(_fromUtf8("label_elbowY"))
        self.horizontalLayout_4.addWidget(self.label_elbowY)
        self.Elbow_Y_value = QtGui.QLabel(self.verticalLayoutWidget)
        self.Elbow_Y_value.setMaximumSize(QtCore.QSize(60, 16777215))
        self.Elbow_Y_value.setObjectName(_fromUtf8("Elbow_Y_value"))
        self.horizontalLayout_4.addWidget(self.Elbow_Y_value)
        self.verticalLayout_4.addLayout(self.horizontalLayout_4)
        self.verticalLayout_4.setStretch(0, 1)
        self.verticalLayout_4.setStretch(1, 1)
        self.verticalLayout_4.setStretch(2, 1)
        self.gridLayout.addLayout(self.verticalLayout_4, 0, 0, 1, 1)
        self.connec_disc = QtGui.QPushButton(self.verticalLayoutWidget)
        self.connec_disc.setCheckable(True)
        self.connec_disc.setObjectName(_fromUtf8("connec_disc"))
        self.gridLayout.addWidget(self.connec_disc, 0, 1, 1, 1)
        self.verticalLayout_5 = QtGui.QVBoxLayout()
        self.verticalLayout_5.setSizeConstraint(QtGui.QLayout.SetNoConstraint)
        self.verticalLayout_5.setObjectName(_fromUtf8("verticalLayout_5"))
        self.checkBox_gripper = QtGui.QCheckBox(self.verticalLayoutWidget)
        self.checkBox_gripper.setMaximumSize(QtCore.QSize(160, 16777215))
        self.checkBox_gripper.setObjectName(_fromUtf8("checkBox_gripper"))
        self.verticalLayout_5.addWidget(self.checkBox_gripper)
        self.horizontalLayout_5 = QtGui.QHBoxLayout()
        self.horizontalLayout_5.setSizeConstraint(QtGui.QLayout.SetFixedSize)
        self.horizontalLayout_5.setObjectName(_fromUtf8("horizontalLayout_5"))
        self.max_open_gripper = QtGui.QLabel(self.verticalLayoutWidget)
        self.max_open_gripper.setMaximumSize(QtCore.QSize(160, 16777215))
        self.max_open_gripper.setObjectName(_fromUtf8("max_open_gripper"))
        self.horizontalLayout_5.addWidget(self.max_open_gripper)
        self.Gripper_maxvalue = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.Gripper_maxvalue.setMaximumSize(QtCore.QSize(160, 16777215))
        self.Gripper_maxvalue.setObjectName(_fromUtf8("Gripper_maxvalue"))
        self.horizontalLayout_5.addWidget(self.Gripper_maxvalue)
        self.verticalLayout_5.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_6 = QtGui.QHBoxLayout()
        self.horizontalLayout_6.setSizeConstraint(QtGui.QLayout.SetFixedSize)
        self.horizontalLayout_6.setObjectName(_fromUtf8("horizontalLayout_6"))
        self.min_close_gripper = QtGui.QLabel(self.verticalLayoutWidget)
        self.min_close_gripper.setMaximumSize(QtCore.QSize(160, 16777215))
        self.min_close_gripper.setObjectName(_fromUtf8("min_close_gripper"))
        self.horizontalLayout_6.addWidget(self.min_close_gripper)
        self.Gripper_minvalue = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.Gripper_minvalue.setMaximumSize(QtCore.QSize(160, 16777215))
        self.Gripper_minvalue.setObjectName(_fromUtf8("Gripper_minvalue"))
        self.horizontalLayout_6.addWidget(self.Gripper_minvalue)
        self.verticalLayout_5.addLayout(self.horizontalLayout_6)
        self.verticalLayout_5.setStretch(0, 2)
        self.verticalLayout_5.setStretch(1, 1)
        self.verticalLayout_5.setStretch(2, 1)
        self.gridLayout.addLayout(self.verticalLayout_5, 0, 2, 1, 1)
        self.gridLayout.setColumnStretch(0, 1)
        self.verticalLayout.addLayout(self.gridLayout)
        self.verticalLayout.setStretch(0, 1)
        self.verticalLayout.setStretch(1, 1)
        self.verticalLayout.setStretch(2, 1)
        self.verticalLayout.setStretch(3, 1)
        self.verticalLayout.setStretch(4, 4)
        Control_Poppy_Right_Arm.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(Control_Poppy_Right_Arm)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 776, 21))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        Control_Poppy_Right_Arm.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(Control_Poppy_Right_Arm)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        Control_Poppy_Right_Arm.setStatusBar(self.statusbar)

        self.retranslateUi(Control_Poppy_Right_Arm)
        QtCore.QObject.connect(self.Slider_X, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.X_value.setNum)
        QtCore.QObject.connect(self.Slider_Y, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.Y_value.setNum)
        QtCore.QObject.connect(self.Slider_Z, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.Z_value.setNum)
        QtCore.QObject.connect(self.Slider_wrist, QtCore.SIGNAL(_fromUtf8("valueChanged(int)")), self.Wrist_value.setNum)
        QtCore.QMetaObject.connectSlotsByName(Control_Poppy_Right_Arm)

    def retranslateUi(self, Control_Poppy_Right_Arm):
        Control_Poppy_Right_Arm.setWindowTitle(_translate("Control_Poppy_Right_Arm", "Control Poppy Right Arm", None))
        self.X.setText(_translate("Control_Poppy_Right_Arm", "X", None))
        self.X_value.setText(_translate("Control_Poppy_Right_Arm", "0", None))
        self.Y.setText(_translate("Control_Poppy_Right_Arm", "Y", None))
        self.Y_value.setText(_translate("Control_Poppy_Right_Arm", "0", None))
        self.Z.setText(_translate("Control_Poppy_Right_Arm", "Z", None))
        self.Z_value.setText(_translate("Control_Poppy_Right_Arm", "0", None))
        self.Wrist.setText(_translate("Control_Poppy_Right_Arm", "Wrist", None))
        self.Wrist_value.setText(_translate("Control_Poppy_Right_Arm", "0", None))
        self.label_shoulderY.setText(_translate("Control_Poppy_Right_Arm", "Shoulder_Y angle:", None))
        self.Shoulder_Y_value.setText(_translate("Control_Poppy_Right_Arm", "0", None))
        self.label_shoulderX.setText(_translate("Control_Poppy_Right_Arm", "Shoulder_X angle:", None))
        self.Shoulder_X_value.setText(_translate("Control_Poppy_Right_Arm", "0", None))
        self.label_elbowY.setText(_translate("Control_Poppy_Right_Arm", "Elbow_Y angle     :", None))
        self.Elbow_Y_value.setText(_translate("Control_Poppy_Right_Arm", "0", None))
        self.connec_disc.setText(_translate("Control_Poppy_Right_Arm", "Connect", None))
        self.checkBox_gripper.setText(_translate("Control_Poppy_Right_Arm", "Open/Close Gripper", None))
        self.max_open_gripper.setText(_translate("Control_Poppy_Right_Arm", "Max open gripper", None))
        self.min_close_gripper.setText(_translate("Control_Poppy_Right_Arm", "Min close gripper", None))