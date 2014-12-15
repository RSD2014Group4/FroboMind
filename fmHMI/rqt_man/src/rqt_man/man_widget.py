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
from std_msgs.msg import Bool

#fmCommand/deadman

class ManWidget(QWidget):
    def __init__(self):
        super(ManWidget, self).__init__()
	rospy.init_node('gui_TurboUlla', anonymous=True)
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_man'), 'resource', 'man_widget.ui')
        loadUi(ui_file, self)
        self.setObjectName('ManWidget')

        self.pushButtonAuto.clicked.connect(self._handle_auto_clicked)
        self.pushButtonMan.clicked.connect(self._handle_man_clicked)
	self.manualMode = True
	self.pushButtonMan.setStyleSheet("background-color: green")

        self.pushButtonTpRS.clicked.connect(self._handle_tp_R_S_clicked)
	
	self.pushButtonDeadman.clicked.connect(self._handle_deadman_clicked)
	self.pushButtonDeadman.setStyleSheet("color: black; background-color: red")
	self.deadman_state = False

	self.deadman_topic = rospy.get_param("deadman_pub", "/fmCommand/deadman")
	self.deadman_msg = Bool()
	self.deadman_pub = rospy.Publisher('deadman_topic', Bool, queue_size=10)
	rospy.Timer(rospy.Duration(0.05), self.deadmanUpdater)

	self.labelName1.setText("Total Logging time: ")
	self.labelName2.setText("Active time: ")
	self.labelName3.setText("Effective prod. from log time: ")
	self.labelName4.setText("Effective prod. from active time: ")
	self.labelName5.setText("Man. from active time: ")
	self.labelName6.setText("Waiting in active time: ")
	self.labelName7.setText("Overall Equipment Efficiency: ")
	self.labelName8.setText("Battery voltage: ")

	rospy.Subscriber("oee_data", oee_data, self.updateOEE)

	self.tipperClient =  actionlib.SimpleActionClient('tipper_action',  tipperAction)

	# Initialize publisher
	self.pub = rospy.Publisher('gui_command', gui_command, queue_size=1)

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
        
    def save_settings(self, settings):
        pass

    def restore_settings(self, settings):
        pass

