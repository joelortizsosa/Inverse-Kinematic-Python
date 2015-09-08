#! /usr/bin/env python
# -*- coding: utf-8 -*-
import sys #on importe le module sys
from PyQt4 import QtGui #on importe le module QtGui 
import QTPoppy as IHM
from poppy.creatures import PoppyRightArm
#from poppy.creatures import PoppyHumanoid
from time import sleep
import numpy as np
import requests , math, cmath, json
from PyQt4.QtGui import * 
from PyQt4.QtCore import * 

def init_arm():
     poppy=PoppyRightArm()
#     poppy=PoppyHumanoid(simulator='vrep')
     sleep(0.5)
     poppy.compliant = False
     poppy.power_up()
# Change PID of Dynamixel motors
     for m in filter(lambda m: hasattr(m, 'pid'), poppy.motors):
         m.pid = (1.9, 5, 0)
# Change PID of XL-320 motors
     for m in poppy.r_gripper:
         m.pid = (4, 2, 0)
# Change PID of Gripper (r_m5)
     poppy.r_m5.pid=(8,0,0)
# Reduce max torque to keep motor temperature low
     for m in poppy.motors:
         m.torque_limit = 70
# Initialize the angles of the motors
     poppy.r_shoulder_x.goal_position =-20
     poppy.r_m1.goal_position =0
     poppy.r_m4.goal_position =90
     print 'Poppy is alive!'
     return poppy

def cadena_4char(x):
    opStr=str(x)
    mostrar='{0:4d}'.format(int(x)).replace(' ','0')
    return mostrar

def robot3inv(x,y,z):
    x=0.1 if x==0 else x
    y=0.1 if y==0 else y
    z=0.1 if z==0 else z

    # geometrie du robot
    l1=16.0
    l2=20.0

    Cq2 = (x**2 + y**2 + z**2 - l1**2 - l2**2)/(2*l1*l2)
    
    q2=-np.emath.arccos(Cq2)
    q1=math.atan2(z,np.lib.scimath.sqrt(x**2 + y**2))-cmath.atan(-l2*np.lib.scimath.sqrt(1-Cq2**2)/(l1+l2*Cq2))
    q0=-math.atan2(x,y)
    
    #passage en degre
    q2 = round((q2.real*180/math.pi),2); # l_elbow_y
    q1 = -round((q1.real*180/math.pi),2);   # l_shoulder_x
    q0 = round((q0.real*180/math.pi),2);   # l_shoulder_y

    return q0,q1,q2

#On crée une classe qui hérite de la classe QtGui.QMainWindow
class MaFenetre(QtGui.QMainWindow,IHM.Ui_Control_Poppy_Right_Arm):
    # on définit la méthode spéciale "init" qui est exécutée
    # lors de l'instanciation de la classe "MaFenetre"
    def __init__(self):
        super(MaFenetre,self).__init__() #on exécute la méthode spéciale "init" des classes héritées
        self.setupUi(self)
        self.onInit() # on apppelle une méthode pour créer les objets/widgets
        self.show() # on montre notre UI
        
    def onInit(self):
        #on écrit le corps de notre interface avant de la montrer

        self.connec_disc.clicked.connect(self.connectDisconnect)
        self.Gripper_minvalue.setText("0")
        self.Gripper_maxvalue.setText("25")
    
    def connectDisconnect(self):
        if self.connec_disc.isChecked():
            self.connec_disc.setText('Disconnect')			
        else:
            self.connec_disc.setText('Connect')
			
    def closeEvent(self,event):
        print "good bye"
#        self.leThreadDeCom.exit()
        
class myLCDNumber(QLCDNumber):

  @pyqtSlot()
  def count(self):
    ihm=fenetre 
    # verification de la proprete du timer 
    if ihm.connec_disc.isChecked():
        # restriction in the space exploration
        if ihm.Slider_Z < 60:
            if ihm.Slider_X > 0 and ihm.Slider_X <=60:
                ihm.Slider_X.setValue=60
                ihm.Slider_X.setMinimum(60)
                ihm.Slider_X.setMaximum(360)
            elif ihm.Slider_X < 0 and ihm.Slider_X >=-60:
                ihm.Slider_X.setValue=-60
                ihm.Slider_X.setMinimum(-60)
                ihm.Slider_X.setMaximum(-360)
            else:
                ihm.Slider_X.setMinimum(-360)
                ihm.Slider_X.setMaximum(360)
                
        # Show values of the Sliders
        ihm.X_value.setText(str(ihm.Slider_X.value()/float(10)))
        ihm.Y_value.setText(str(ihm.Slider_Y.value()/float(10)))
        ihm.Z_value.setText(str(ihm.Slider_Z.value()/float(10)))
        # obtaining angles for motors r_m2 and r_m3                
        ecu=np.array([(ihm.Slider_Y.value()/float(10)),(ihm.Slider_Z.value()/float(10)),(ihm.Slider_Y.value()/float(10))*(ihm.Slider_Z.value()/float(10)),(ihm.Slider_Y.value()/float(10))**2,(ihm.Slider_Z.value()/float(10))**2])
        ang12=np.dot(np.insert(ecu,0,1),j)
                    
        #obtaining angles through inverse kinematics
        [l_shoulder_y,l_shoulder_x,l_elbow_y]=robot3inv( (ihm.Slider_X.value()/float(10)),
                                                         (ihm.Slider_Y.value()/float(10)),
                                                         (ihm.Slider_Z.value()/float(10)) )
        motor_l_shoulder_y=cadena_4char(l_shoulder_y)
        ihm.Shoulder_Y_value.setText(str(motor_l_shoulder_y))
        motor_l_shoulder_x=cadena_4char(l_shoulder_x)
        ihm.Shoulder_X_value.setText(str(motor_l_shoulder_x))
        motor_l_elbow_y   =cadena_4char(l_elbow_y)
        ihm.Elbow_Y_value.setText(str(motor_l_elbow_y))
        #open/close gripper
        if ihm.checkBox_gripper.isChecked():
            motor_r_m5 = round(int(ihm.Gripper_minvalue.text()))
        else:
            motor_r_m5 = round(int(ihm.Gripper_maxvalue.text()))
                
        # poignet
        poignet = ihm.Slider_wrist.value()
        #envoi
        poppy.r_shoulder_y.goal_position = l_shoulder_y
        poppy.r_shoulder_x.goal_position =l_shoulder_x
        poppy.r_arm_z.goal_position = -90
        poppy.r_elbow_y.goal_position =l_elbow_y				
        poppy.r_m2.goal_position =ang12[0]
        poppy.r_m3.goal_position =ang12[1]
        poppy.r_m4.goal_position =round(poignet)
        poppy.r_m5.goal_position =motor_r_m5	
     

if __name__=='__main__':

    j=np.array([[42.0065735815700,1.88688034226815],[-0.299041316824250,-4.10873223361288],[-3.11708848964619,-0.0778139097247628],[0.0533663361088492,-0.00204656981431027],[-0.0233476775151460,0.0577419354177804],[-0.00234324559533085,-0.000253340560084825]])    
    app=QtGui.QApplication(sys.argv) # on crée un objet application
    poppy=init_arm()	
    lcdNumber	 = myLCDNumber()
    fenetre=MaFenetre() # on instancie la classe MaFenetre
    timer = QTimer()
    lcdNumber.connect(timer,SIGNAL("timeout()"),lcdNumber,SLOT("count()"))
    timer.start(0.001)       
    sys.exit(app.exec_()) # on quitte correctement