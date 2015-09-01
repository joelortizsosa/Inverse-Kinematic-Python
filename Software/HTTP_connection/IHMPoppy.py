#! /usr/bin/env python
# -*- coding: utf-8 -*-
import sys #on importe le module sys
from PyQt4 import QtGui #on importe le module QtGui 
import QTPoppy as IHM
from poppy.creatures import PoppyRightArm
from threading import Thread
from time import sleep
import numpy as np
import requests , math, cmath, json

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
    q2 = round((q2.real*180/math.pi)); # l_elbow_y
    q1 = -round(q1.real*180/math.pi);   # l_shoulder_x
    q0 = round(q0.real*180/math.pi);   # l_shoulder_y

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
		self.leThreadDeCom.exit()

class MonThread(Thread):
    def __init__(self,fenetre):
        super(MonThread,self).__init__()
        self.ihm=fenetre
        self.j=np.array([[42.0065735815700,1.88688034226815],[-0.299041316824250,-4.10873223361288],[-3.11708848964619,-0.0778139097247628],[0.0533663361088492,-0.00204656981431027],[-0.0233476775151460,0.0577419354177804],[-0.00234324559533085,-0.000253340560084825]])

    def run(self):
        while 1:
			#print 't'
			#print self.ihm.connec_disc.isChecked()
            if self.ihm.connec_disc.isChecked():
                # restriction in the space exploration
                if self.ihm.Slider_Z < 6:
                    if self.ihm.Slider_X > 0 and self.ihm.Slider_X <=6:
                        self.ihm.Slider_X.setValue=6
                        self.ihm.Slider_X.setMinimum(6)
                        self.ihm.Slider_X.setMaximum(36)
                    elif self.ihm.Slider_X < 0 and self.ihm.Slider_X >=-6:
                        self.ihm.Slider_X.setValue=-6
                        self.ihm.Slider_X.setMinimum(-6)
                        self.ihm.Slider_X.setMaximum(-36)
                else:
                    self.ihm.Slider_X.setMinimum(-36)
                    self.ihm.Slider_X.setMaximum(36)
                
                # obtaining angles for motors r_m2 and r_m3                
                ecu=np.array([self.ihm.Slider_Y.value(),self.ihm.Slider_Z.value(),self.ihm.Slider_Y.value()*self.ihm.Slider_Z.value(),self.ihm.Slider_Y.value()**2,self.ihm.Slider_Z.value()**2])
                ang12=np.dot(np.insert(ecu,0,1),self.j)
                
                #obtaining angles through inverse kinematics
                [l_shoulder_y,l_shoulder_x,l_elbow_y]=robot3inv(self.ihm.Slider_X.value(),
                                                                self.ihm.Slider_Y.value(),
                                                                self.ihm.Slider_Z.value())
                motor_l_shoulder_y=cadena_4char(l_shoulder_y)
                self.ihm.Shoulder_Y_value.setText(str(motor_l_shoulder_y))
                motor_l_shoulder_x=cadena_4char(l_shoulder_x)
                self.ihm.Shoulder_X_value.setText(str(motor_l_shoulder_x))
                motor_l_elbow_y   =cadena_4char(l_elbow_y)
                self.ihm.Elbow_Y_value.setText(str(motor_l_elbow_y))
                print l_shoulder_x
                #open/close gripper
                if self.ihm.checkBox_gripper.isChecked():
                    motor_r_m5 = round(int(self.ihm.Gripper_minvalue.text()))
                else:
                    motor_r_m5 = round(int(self.ihm.Gripper_maxvalue.text()))
                
                # poignet
                poignet = self.ihm.Slider_wrist.value()
                
                #envoi
                requests.post('http://127.0.0.1:3030/motor/r_elbow_y/register/goal_position/value.json',
                              data=json.dumps(l_elbow_y), headers={'content-type': 'application/json'})
                requests.post('http://127.0.0.1:3030/motor/r_shoulder_x/register/goal_position/value.json',
                              data=json.dumps(l_shoulder_x), headers={'content-type': 'application/json'})
                requests.post('http://127.0.0.1:3030/motor/r_shoulder_y/register/goal_position/value.json',
                              data=json.dumps(l_shoulder_y), headers={'content-type': 'application/json'})
                requests.post('http://127.0.0.1:3030/motor/r_arm_z/register/goal_position/value.json',
                              data=json.dumps(-90), headers={'content-type': 'application/json'})
                requests.post('http://127.0.0.1:3030/motor/r_m2/register/goal_position/value.json',
                              data=json.dumps(ang12[0]), headers={'content-type': 'application/json'})
                requests.post('http://127.0.0.1:3030/motor/r_m3/register/goal_position/value.json',
                              data=json.dumps(ang12[1]), headers={'content-type': 'application/json'})
                requests.post('http://127.0.0.1:3030/motor/r_m4/register/goal_position/value.json',
                              data=json.dumps(round(poignet)), headers={'content-type': 'application/json'})
                requests.post('http://127.0.0.1:3030/motor/r_m5/register/goal_position/value.json',
                              data=json.dumps(motor_r_m5), headers={'content-type': 'application/json'})
                #reception
                angleWrist = requests.get('http://127.0.0.1:3030/motor/r_m4/register/present_position')
                #print(angleWrist.json())

                sleep(0.04)

if __name__=='__main__':
    app=QtGui.QApplication(sys.argv) # on crée un objet application
    fenetre=MaFenetre() # on instancie la classe MaFenetre
    comThread=MonThread(fenetre)
    fenetre.leThreadDeCom=comThread
    comThread.start()
    sys.exit(app.exec_()) # on quitte correctement