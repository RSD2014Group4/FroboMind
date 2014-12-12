#!/usr/bin/env python
# imports
import rospy
import pickle 
import datetime
import os.path
from smach_msgs.msg import SmachContainerStatus

class oeeNode():
	def __init__(self):

	#defines
		# Time in different catagories
		self.stateTimes = {"totalTimePeriod": datetime.timedelta(0), "shutDownTime": datetime.timedelta(0), "manual": datetime.timedelta(0), "charging": datetime.timedelta(0), "linefollowing": datetime.timedelta(0), "dispenserZone": datetime.timedelta(0), "robotUnload": datetime.timedelta(0), "robotLoad": datetime.timedelta(0)} # " tid mellem "totalTimePeriod" og summen af de andre er spild
		self.currentState = "manual"
		self.currentTimeStamp = datetime.datetime.now()
		self.nxtTimeStamp = datetime.datetime.now()

		# Load old values from file
		self.loadTimesFromFile()

		# Update frequency 
		self.updateInterval = 0.1 #[Hz]
		self.setupOeeUpdateTimer()

		# Timer to save oee to file
		self.saveInterval = 60 #[s]
		self.setupSaveFileTimer()

		# Go to listener
		self.setupListener()

		# Go to infinity loop
		rospy.loginfo(rospy.get_name() + " Enter while infinity: %s"%datetime.datetime.now())
#		self.updater()
		rospy.spin()

	def setupOeeUpdateTimer(self):
		rospy.Timer(rospy.Duration(self.updateInterval), self._callbackOeeUpdate)

	def _callbackOeeUpdate(self):
		#send parameters via msg 
		rospy.loginfo(rospy.get_name() + ": Update oee para! ")
	
	def setupSaveFileTimer(self):
		rospy.Timer(rospy.Duration(self.saveInterval), self._callback)

	def _callback(self,x):
		self.saveToFile()
		rospy.loginfo(rospy.get_name() + ": Save file! ")
#		print x

	def saveToFile(self):
		self.stateTimes["shutDownTime"] = datetime.datetime.now()
		rospy.loginfo(rospy.get_name() + ": Saving to file")
		self._file = open('../date.txt', 'w')
		pickle.dump(self.stateTimes, self._file)
		self._file.close()		

	def setupListener(self):
		rospy.Subscriber("/msTest/smach/container_status", SmachContainerStatus, self.updateSubriberPar)

	def updateSubriberPar(self, x):
		rospy.loginfo(rospy.get_name() + " Update oee!")
		self.nxtTimeStamp = datetime.datetime.now()
		self.updateStateTimes()
		print x

		# manual
		if ('STATE_FOLLOW_LINE_QR') in x.active_states:
			self.currentState = "manual"
			rospy.loginfo(rospy.get_name() + " Hurra vi ramte plet")
			#rospy.loginfo(rospy.get_name() + " Time manual: %s"%self.stateTimes["manual"])
		# charging
#		elif (noget or nogetAndet) in _container:
#			self.currentState = "charging"
		# linefollowing
#		elif (noget or nogetAndet) in _container:
#			self.currentState = "linefollowing"
		# dispensorZone
#		elif (noget or nogetAndet) in _container:
#			self.currentState = "dispenserZone"
		# robotUnload
#		elif (noget or nogetAndet) in _container:
#			self.currentState = "robotUnload"
		# robotLoad
#		elif (noget or nogetAndet) in _container:
#			self.currentState = "robotLoad"

		self.currentTimeStamp = self.nxtTimeStamp

	def updateStateTimes(self):
		if self.currentState == "manual":
			self.stateTimes["manual"] = self.stateTimes["manual"] + (self.nxtTimeStamp - self.currentTimeStamp)
		elif self.currentState == "charging":
			self.stateTimes["charging"] = self.stateTimes["charging"] + (self.nxtTimeStamp - self.currentTimeStamp)
		#osv (Skal skrives faerdig naar state kendes)

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
		else:
			self.time_last_file = open('../date.txt', 'r')
		self.stateTimes = pickle.load(self.time_last_file)
		self.time_last_file.close()
#		rospy.loginfo(rospy.get_name() + ": Last shut down time: " + str(self.buffer))
		

#Main function
if __name__ == '__main__':
	# Initialize the node and name it.
	rospy.init_node('oee_cal_node', anonymous=True)

	# Go to class functions that do all the heavy lifting. Do error checking.
	try:
		node_class = oeeNode()
	except rospy.ROSInterruptException:
		pass


