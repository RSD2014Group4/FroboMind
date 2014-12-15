#!/usr/bin/env python

import os
import rospkg
import rospy
import actionlib

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QUrl
from python_qt_binding.QtGui import QCompleter, QIcon, QWidget, QTableWidgetItem
from python_qt_binding.QtWebKit import QWebPage, QWebView
from TurboUlla.msg import oee_data
from rqt_man.msg import gui_command
from lift_tipper.msg import tipperAction, tipperGoal
from std_msgs.msg import Bool #, Int
from PyQt4 import QtGui, QtCore
from geometry_msgs.msg import PoseWithCovarianceStamped

class ManWidget(QWidget):
    def __init__(self):
        super(ManWidget, self).__init__()
        rospy.init_node('gui_TurboUlla', anonymous=True)
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_man'), 'resource', 'man_widget.ui')
        loadUi(ui_file, self)
        self.pathArrows = "../rsd_roswork/src/fmHMI/rqt_man/resource/leftDown.png"
        self.pathArrows2 = "../rsd_roswork/src/fmHMI/rqt_man/resource/left.png"
        self.setObjectName('ManWidget')

        #Man auto reset
        self.pushButtonAuto.clicked.connect(self._handle_auto_clicked)
        self.pushButtonMan.clicked.connect(self._handle_man_clicked)
        self.manualMode = True
        self.pushButtonMan.setStyleSheet("background-color: green")
    
        #Tipper
        self.pushButtonTpRS.clicked.connect(self._handle_tp_R_S_clicked)
        self.tipperClient =  actionlib.SimpleActionClient('tipper_action',  tipperAction)
    	
        #Deadman
        self.pushButtonDeadman.clicked.connect(self._handle_deadman_clicked)
        self.pushButtonDeadman.setStyleSheet("color: black; background-color: red")
        self.deadman_state = False
        self.deadman_topic = rospy.get_param("deadman_pub", "/fmCommand/deadman")
        self.deadman_msg = Bool()
        self.deadman_pub = rospy.Publisher(self.deadman_topic, Bool, queue_size=10)
        rospy.Timer(rospy.Duration(0.05), self.deadmanUpdater)
    
        # OEE data
        self.labelName1.setText("Total Logging time: ")
        self.labelName2.setText("Active time: ")
        self.labelName3.setText("Effective prod. from log time: ")
        self.labelName4.setText("Effective prod. from active time: ")
        self.labelName5.setText("Man. from active time: ")
        self.labelName6.setText("Waiting in active time: ")
        self.labelName7.setText("Overall Equipment Efficiency: ")
        self.labelName8.setText("Battery voltage: ")
        rospy.Subscriber("oee_data", oee_data, self.updateOEE)
    
        self.InitPackML()
    
        #Feedback position
        self.pushCharger1.clicked.connect(self.charger_1_clicked)
        self.pushCharger2.clicked.connect(self.charger_2_clicked)
        self.pushCharger3.clicked.connect(self.charger_3_clicked)
        self.positionFromMan = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.startPos = PoseWithCovarianceStamped()
    
#        self.pushButton_4.clicked.connect(self.action_find_pos_clicked)
    
        # Error handling
        self.pushReset.clicked.connect(self.reset_clicked)
        self.pushReset.setStyleSheet("background-color: red")
        self.error = True
    
        # Initialize publisher
        self.pub = rospy.Publisher('gui_command', oee_data, queue_size=1)

	# Battery voltage
#	rospy.Subscriber("/battery", Int, self.updateBatteryVoltage)

#    def updateBatteryVoltage(self, data):
#	self.labelValue8.setText(str(data/10))
    
    def reset_clicked(self):
        self.pushReset.setStyleSheet("background-color: light grey")
        self.error = False
        
    
    def charger_1_clicked(self):
            self.startPos.header.frame_id = "map"
            self.startPos.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
            self.startPos.pose.pose.position.x = 4.38809776306
            self.startPos.pose.pose.position.y = 0.832479357719
            self.startPos.pose.pose.position.z = 0.0
            self.startPos.pose.pose.orientation.x = 0.0
            self.startPos.pose.pose.orientation.y = 0.0
            self.startPos.pose.pose.orientation.z = 0.703081068125
            self.startPos.pose.pose.orientation.w = 0.711109704366
            self.positionFromMan.publish (self.startPos) 
     
    def charger_2_clicked(self):
            self.startPos.header.frame_id = "map"
            self.startPos.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
            self.startPos.pose.pose.position.x = 4.86667108536
            self.startPos.pose.pose.position.y = 0.756682872772
            self.startPos.pose.pose.position.z = 0.0
            self.startPos.pose.pose.orientation.x = 0.0
            self.startPos.pose.pose.orientation.y = 0.0
            self.startPos.pose.pose.orientation.z = 0.698544527071
            self.startPos.pose.pose.orientation.w = 0.715566589284
            self.positionFromMan.publish (self.startPos) 
    
    def charger_3_clicked(self):
            self.startPos.header.frame_id = "map"
            self.startPos.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
            self.startPos.pose.pose.position.x = 5.47654533386
            self.startPos.pose.pose.position.y = 0.81746673584
            self.startPos.pose.pose.position.z = 0.0
            self.startPos.pose.pose.orientation.x = 0.0
            self.startPos.pose.pose.orientation.y = 0.0
            self.startPos.pose.pose.orientation.z = 0.684241281656
            self.startPos.pose.pose.orientation.w = 0.729255694855
            self.positionFromMan.publish (self.startPos) 
    
    def deadmanUpdater(self, event):
    		self.deadman_msg = self.deadman_state
    		self.deadman_pub.publish (self.deadman_msg)
    
    def updateOEE(self, data):
            self.labelValue1.setText(data.logfileTime + ' [t]')
            self.labelValue2.setText(data.ActiveTime + ' [t]')
            self.labelValue3.setText(data.prodPercentageFromLogfileStart + ' [%]')
            self.labelValue4.setText(data.prodEffPercentActiveRobot + ' [%]')
            self.labelValue5.setText(data.manualPercentActiveRobot + ' [%]')
            self.labelValue6.setText(data.waitingPercentActiveRobot + ' [%]')
            self.labelValue7.setText(data.OEE )
    
    def _handle_deadman_clicked(self):
            if self.deadman_state == False:
                self.deadman_state = True
                self.pushButtonDeadman.setStyleSheet("color: black; background-color: green")
                self.pushButtonDeadman.setText('Deadman active')
            else:
                self.deadman_state = False
                self.pushButtonDeadman.setStyleSheet("color: black; background-color: red")
                self.pushButtonDeadman.setText('Deadman inactive')
    	
    def _handle_auto_clicked(self):
        if self.error == False:
            self.pushButtonMan.setStyleSheet("background-color: light grey")
            self.pushButtonAuto.setStyleSheet("background-color: green")
            self.manualMode = False
            self.msg = gui_command()
            self.msg.command = self.msg.COMMAND_AUTO
            self.pub.publish(self.msg)
    
    def _handle_man_clicked(self):
            self.pushButtonAuto.setStyleSheet("background-color: light grey")
            self.pushButtonMan.setStyleSheet("background-color: green")
    
            self.msg = gui_command()
            self.msg.command = self.msg.COMMAND_MAN	
            self.pub.publish(self.msg)
            self.manualMode = True
    
    
    def _handle_tp_R_S_clicked(self):
            if self.manualMode == True:
                rospy.loginfo("Tipping")
                goal = tipperGoal()
                goal.lift = True;
                self.tipperClient.send_goal(goal)
                self.tipperClient.wait_for_result(rospy.Duration.from_sec(10))
    
    def InitPackML(self):
            self.textBrowIdle.setText("Get position")
            self.textBrowIdle.setAlignment(Qt.AlignCenter)
            self.textBrowIdle.setMaximumHeight(30)
    
            self.textBrowMan.setText("Manual")
            self.textBrowMan.setAlignment(Qt.AlignCenter)
            self.textBrowMan.setMaximumHeight(30)
    
            self.textBrowReset.setText("Resetting")
            self.textBrowReset.setAlignment(Qt.AlignCenter)
            self.textBrowReset.setMaximumHeight(30)
    
            self.textBrowAuto.setText("Automatic")
            self.textBrowAuto.setAlignment(Qt.AlignCenter)
            self.textBrowAuto.setMaximumHeight(30)
    	
            self.textBrowAbort.setText("Aborting")
            self.textBrowAbort.setAlignment(Qt.AlignCenter)
            self.textBrowAbort.setMaximumHeight(30)
    	
            self.textBrowError.setText("Error")
            self.textBrowError.setAlignment(Qt.AlignCenter)
            self.textBrowError.setMaximumHeight(30)
    
            self.labelArUp1.setAlignment(Qt.AlignCenter)
            self.labelArUp2.setAlignment(Qt.AlignCenter)
            self.labelArRight1.setAlignment(Qt.AlignCenter)
            self.labelArLeftDown1.setAlignment(Qt.AlignCenter)
            self.labelArLeft1.setAlignment(Qt.AlignCenter)
            self.labelArDown1.setAlignment(Qt.AlignCenter)
            self.labelArDown2.setAlignment(Qt.AlignCenter)
    
            self.pixmap = QtGui.QPixmap(self.pathArrows)
            self.labelArLeftDown1.setPixmap(self.pixmap.scaled (50, 50, QtCore.Qt.KeepAspectRatio))
    
            self.pixmap = QtGui.QPixmap(self.pathArrows2)
            self.pixmap = self.pixmap.scaled (50, 50, QtCore.Qt.KeepAspectRatio)
            self.pixmap = self.pixmap.transformed(QtGui.QTransform().rotate(0))
            self.labelArLeft1.setPixmap(self.pixmap)
    	
            #self.pixmap = QtGui.QPixmap(self.pathArrows2)
            self.pixmap = self.pixmap.scaled (50, 50, QtCore.Qt.KeepAspectRatio)
            self.pixmap = self.pixmap.transformed(QtGui.QTransform().rotate(90))
            self.labelArUp1.setPixmap(self.pixmap)
            self.labelArUp2.setPixmap(self.pixmap)
    
            self.pixmap = self.pixmap.transformed(QtGui.QTransform().rotate(90))
            self.labelArRight1.setPixmap(self.pixmap)
    
            self.pixmap = self.pixmap.transformed(QtGui.QTransform().rotate(90))
            self.labelArDown1.setPixmap(self.pixmap)
            self.labelArDown2.setPixmap(self.pixmap)
    
    def save_settings(self, settings):
            pass
    
    def restore_settings(self, settings):
            pass
    
