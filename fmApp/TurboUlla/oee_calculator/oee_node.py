#!/usr/bin/env python
# imports
import rospy
import pickle 
import datetime
import os.path
from TurboUlla.msg import oee_data
from smach_msgs.msg import SmachContainerStatus



class oeeNode():
	def __init__(self):

	#defines
		# Time in different catagories
		self.stateTimes = {"totalTimePeriod": datetime.timedelta(0), "manual": datetime.timedelta(0), "production": datetime.timedelta(0), "waiting": datetime.timedelta(0)} 

		self.currentState = "None"
		self.currentTimeStamp = datetime.datetime.now()
		self.nxtTimeStamp = datetime.datetime.now()

		# Load old values from file
		self.loadTimesFromFile()

		# Initialize publisher
		self.pub = rospy.Publisher('oee_data', oee_data, queue_size=10)

		# Timer to save oee to file
		self.saveInterval = 60 #[s]
		self.setupSaveFileTimer()

		# Go to listener
		self.setupListener()

		# Go to infinity loop
		rospy.spin()
	
	def setupSaveFileTimer(self):
		rospy.Timer(rospy.Duration(self.saveInterval), self._callback)

	def _callback(self,x):
		self.saveToFile()
		rospy.loginfo(rospy.get_name() + ": Save file! ")
#		print x

	def saveToFile(self):
		rospy.loginfo(rospy.get_name() + ": Saving to file")
		self._file = open('../date.txt', 'w')
		pickle.dump(self.stateTimes, self._file)
		self._file.close()

	def setupListener(self):
		rospy.Subscriber("/decisionLayerIntrospectionServer/smach/container_status", SmachContainerStatus, self.updateSubPar)

	def updateSubPar(self, x):
		rospy.loginfo(rospy.get_name() + " Update oee!")
		self.nxtTimeStamp = datetime.datetime.now()
		self.updateStateTimes()
		self._container = x.active_states

		# manual
		if ('STATE_IN_MANUAL_MODE') in self._container:
			self.currentState = "manual"
		# production
		elif ('STATE_NAVIGATE_IN_LINE_ZONE' in self._container or 'STATE_NAVIGATE_IN_COORDINATE_ZONE' in self._container or 'STATE_NAVIGATE_IN_LOAD_ZONE' in self._container) :
			self.currentState = "production"
		# wait
		elif ('STATE_FREE_AT_LINE_ZONE' in self._container or 'STATE_FREE_AT_LOAD_ZONE' in self._container or 'STATE_FREE_AT_COORDINATE_ZONE' in self._container) :
			self.currentState = "waiting"
		else:
			self.currentState = "None"
		
		self.currentTimeStamp = self.nxtTimeStamp

	def updateStateTimes(self):
		if self.currentState == "manual":
			self.stateTimes["manual"] = self.stateTimes["manual"] + (self.nxtTimeStamp - self.currentTimeStamp)
		elif self.currentState == "production":
			self.stateTimes["production"] = self.stateTimes["production"] + (self.nxtTimeStamp - self.currentTimeStamp)
		elif self.currentState == "waiting":
			self.stateTimes["waiting"] = self.stateTimes["waiting"] + (self.nxtTimeStamp - self.currentTimeStamp)

		self.updateOEEPar()
	
	def updateOEEPar(self):
		self.totalTimePer = datetime.datetime.now() - self.stateTimes["totalTimePeriod"]
		self.totalActiveTime = (self.stateTimes["manual"] + self.stateTimes["production"] + self.stateTimes["waiting"])
		self.msg = oee_data()

		# Calculations while only considuring the time where the robot is turned on
		if not self.totalActiveTime.total_seconds() == 0:
			self.productionEfficiencyPercentageWhileActive = (self.stateTimes["production"].total_seconds()/self.totalActiveTime.total_seconds())*100
			self.manualPercentageWhileActive = (self.stateTimes["manual"].total_seconds()/self.totalActiveTime.total_seconds())*100
			self.waitingPercentageWhileActive = (self.stateTimes["waiting"].total_seconds()/self.totalActiveTime.total_seconds())*100
			self.msg.ActiveTime = str(self.totalActiveTime)[:str(self.totalActiveTime).index('.')]
			self.msg.prodEffPercentActiveRobot = str(self.productionEfficiencyPercentageWhileActive)[:str(self.productionEfficiencyPercentageWhileActive).index('.')+2]
			self.msg.manualPercentActiveRobot = str(self.manualPercentageWhileActive)[:str(self.manualPercentageWhileActive).index('.')+2]
			self.msg.waitingPercentActiveRobot = str(self.waitingPercentageWhileActive)[:str(self.waitingPercentageWhileActive).index('.')+2]

		# Calculations from logfile is created
		if not self.totalTimePer.total_seconds() == 0:
			self.prodPercentage = (self.stateTimes["production"].total_seconds()/self.totalTimePer.total_seconds())*100
			self.msg.prodPercentageFromLogfileStart = str(self.prodPercentage)[:str(self.prodPercentage).index('.')+2]
			self.msg.logfileTime = str(self.totalTimePer)[:str(self.totalTimePer).index('.')]



		rospy.loginfo(rospy.get_name() + " Time: " + str(self.totalTimePer.total_seconds()))
#		rospy.loginfo(rospy.get_name() + " Active manual percentage: " + str(self.manualPercentageWhileActive))
#		rospy.loginfo(rospy.get_name() + " Active waiting percentage: " + str(self.waitingPercentageWhileActive))

#		rospy.loginfo(rospy.get_name() + " Total time: " + str(self.totalTimePer))
#		rospy.loginfo(rospy.get_name() + " Production time: " + str(self.stateTimes["production"]))
#		rospy.loginfo(rospy.get_name() + " Manual time: " + str(self.stateTimes["manual"]))
#		rospy.loginfo(rospy.get_name() + " Waiting time: " + str(self.stateTimes["waiting"]))

		self.msg.OEE = "0.0"
		self.pub.publish(self.msg)

	# Before deleting -save time to file date.txt
	def __del__(self):
		self.saveToFile()

	# Get time fra last execution at startup
	def loadTimesFromFile(self):
		# Open or create file and open file
		if not os.path.exists('../date.txt'):
			self.time_last_file = open('../date.txt', 'wr')
			self.stateTimes["totalTimePeriod"] = datetime.datetime.now()
			pickle.dump(self.stateTimes, self.time_last_file)
			self.time_last_file.close()
			self.time_last_file = open('../date.txt', 'r')
		else:
			self.time_last_file = open('../date.txt', 'r')
		self.stateTimes = pickle.load(self.time_last_file)
		self.time_last_file.close()

#Main function
if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('oee_cal_node', anonymous=True)

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		node_class = oeeNode()
	except rospy.ROSInterruptException:
		pass


