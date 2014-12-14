#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

import math
import numpy
#import gometry_msgs
import std_msgs

import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from line_til_cross_action.msg import GocellAction, GocellGoal
from line_til_cross_action_backwards.msg import GocellAction as GocellActionBackwards
from line_til_cross_action_backwards.msg import GocellGoal as GocellGoalBackwards

from lift_tipper.msg import tipperAction, tipperGoal


from spin_90_degrees.msg import *


from std_msgs.msg import Bool
#from std_msgs.msg import String
from TurboUlla.msg import mes_mobile_status, mes_mobile_command
from geometry_msgs.msg import Pose, Point, Quaternion 
global r

#TODO: Add subscription to this topic
#set by callback function from navigation topic
global doneNavigating
doneNavigating = False


global debugWait
debugWait = 2.0

global SERVER_WAIT_TIME
SERVER_WAIT_TIME = 2.0

global SERVER_WAIT_TIME_COORD
SERVER_WAIT_TIME_COORD = 60.0

#set from callback function from MES_command topic
global MESCommand
MESCommand = mes_mobile_command.COMMAND_WAIT
#target position/zone
#path = 'FloorOut'
path = 'Station1'
nextPath = 'Station1'
#current position/zone
#position = 'FloorOut'
position = 'Station1'

#TODO: do enum type        
ON = 1
OFF = 0

global goCellClient
global spin90Client
goCellClient = actionlib.SimpleActionClient('Gocell', GocellAction)
goCellBackClient = actionlib.SimpleActionClient('Gocell_back', GocellActionBackwards)
spin90Client = actionlib.SimpleActionClient('spin_degrees', spin_degreesAction)

global tipperClient
tipperClient =  actionlib.SimpleActionClient('tipper_action', tipperAction)

global coordNavClient 
coordNavClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)#Pose)


global cameFromLoadOff
cameFromLoadOff = False
latestRobotCell = "NONE"

#TODO: Dirty code, should not be global variable...
pubStatus = rospy.Publisher("mes_mobile_status", mes_mobile_status, queue_size = 10)
pubNavPose = rospy.Publisher("navigation_goal_pose", Pose, queue_size = 10)
 

#######################################################
#TODO: States should only exit when actually done. DO NOT renter same state simply to wait.
#TODO: 
####################################################       
# define state StateFreeAtCoordinateZone
class StateFreeAtCoordinateZone(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        global MESCommand
        global position
        global r
        global debugWait
        global path
        global nextPath
#        r.sleep()
        rospy.loginfo("currCommand: " + str(MESCommand))
        rospy.loginfo("currPath: " + str(path))    
        rospy.loginfo("currNextPath: " + str(nextPath))
        #If nothing new received from MES, stay free

        rospy.sleep(debugWait)


        while MESCommand == mes_mobile_command.COMMAND_WAIT:
            rospy.loginfo("in COMMAND_WAIT while loop")
            rospy.loginfo("currCommand: " + str(MESCommand))
            rospy.loginfo("currPath: " + str(path))    
            rospy.loginfo("currNextPath: " + str(nextPath))

            #sleep for 5 seconds
            rospy.loginfo('command was wait. 5 sec')
            status = mes_mobile_status()
            status.header.stamp = rospy.Time.now()
            status.state = mes_mobile_status.STATE_FREE
            status.position = position 
            pubStatus.publish(status)
            rospy.sleep(debugWait)
 
        rospy.loginfo("out of COMMAND_WAIT while loop")

        #If command received from MES server, Go to NavigateInCoordinateZone
        if MESCommand == mes_mobile_command.COMMAND_NAVIGATE:
            #TODO:Publish state change to OEE topic
            status = mes_mobile_status()
            status.header.stamp = rospy.Time.now()
            status.state = mes_mobile_status.STATE_WORKING
            status.position = position 
            pubStatus.publish(status)
            
            path = nextPath
            #Reset MESCommand to zero
                #whenever this state, "free", a status have been sent,
                #and a new command will be set by the callback function
            MESCommand = mes_mobile_command.COMMAND_WAIT
            return 'outcome2'  
#        else:
            #ERROR

# define state StateFreeAtLineZone
class StateFreeAtLineZone(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        global MESCommand
        global position
        global r
        global debugWait
        global path
        global nextPath
#        r.sleep()
        
        rospy.sleep(debugWait)
        
        #If nothing new received from MES, stay free
        while MESCommand == mes_mobile_command.COMMAND_WAIT:
            #sleep for 5 seconds
            status = mes_mobile_status()
            status.header.stamp = rospy.Time.now()
            status.state = mes_mobile_status.STATE_FREE
            status.position = position 
            pubStatus.publish(status)
            rospy.sleep(debugWait)
 
        


        
        #If command received from MES server, Go to NavigateInLineZone
        if MESCommand == mes_mobile_command.COMMAND_NAVIGATE:
            #TODO:Publish state change to OEE topic
            status = mes_mobile_status()
            status.header.stamp = rospy.Time.now()
            status.state = mes_mobile_status.STATE_WORKING
            status.position = position 
            pubStatus.publish(status)
            path = nextPath
            #Reset MESCommand to zero
                #whenever this state, "free", a status have been sent,
                #and a new command will be set by the callback function
            MESCommand = mes_mobile_command.COMMAND_WAIT
            return 'outcome2'  
#        else:
            #ERROR


# define state StateFreeAtLoadZone
class StateFreeAtLoadZone(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        global MESCommand
        global position
        global r
#        r.sleep()
        global debugWait
        global path
        global nextPath
        
        rospy.sleep(debugWait)
        
        #If nothing new received from MES, stay free
        while MESCommand == mes_mobile_command.COMMAND_WAIT:
            #sleep for 5 seconds
            status = mes_mobile_status()
            status.header.stamp = rospy.Time.now()
            status.state = mes_mobile_status.STATE_FREE
            status.position = position 
            pubStatus.publish(status)
            rospy.sleep(debugWait)
 

        
        if MESCommand == mes_mobile_command.COMMAND_TIP:
            #TODO:Publish state change to OEE topic
            status = mes_mobile_status()
            status.header.stamp = rospy.Time.now()
            status.state = mes_mobile_status.STATE_WORKING
            status.position = position 
            pubStatus.publish(status)
            
            self.tip() #TODO: CALL ACTION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            status = mes_mobile_status()
            status.header.stamp = rospy.Time.now()
            status.state = mes_mobile_status.STATE_FREE
            status.position = position 
            pubStatus.publish(status)
            
            
            MESCommand = mes_mobile_command.COMMAND_WAIT
            
            return 'outcome1'
        
        #If command received from MES server, Go to NavigateInLoadZone
        if MESCommand == mes_mobile_command.COMMAND_NAVIGATE:
            #TODO:Publish state change to OEE topic
            status = mes_mobile_status()
            status.header.stamp = rospy.Time.now()
            status.state = mes_mobile_status.STATE_WORKING
            status.position = position 
            pubStatus.publish(status)
            
            path = nextPath
            #Reset MESCommand to zero
                #whenever this state, "free", a status have been sent,
                #and a new command will be set by the callback function
            MESCommand = mes_mobile_command.COMMAND_WAIT
            return 'outcome2'  
#        else:
            #ERROR  

    def tip(self):
        rospy.loginfo("Tipping")
        goal = tipperGoal()


        goal.lift = True;
        tipperClient.send_goal(goal)
        tipperClient.wait_for_result(rospy.Duration.from_sec(SERVER_WAIT_TIME))
        rospy.sleep(2)
#        activateTipper()
        return 0
#
#    def tip(self):
##        activateTipper()
#        rospy.sleep(debugWait)
#        rospy.loginfo("\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!TIPPING!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n")
#        return 0

# define state StateNavigateInCoordinateZone
# Uses rabbit line following to navigate a list of internally defined waypoints,
# in order to reach a goal set by the MES server command path.
class StateNavigateInCoordinateZone(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2', 'outcome3'])
        #for now, only a single point is sent as goal. Maybe a list is necesary.
        

    def execute(self, userdata):
        global position
        global r
        global doneNavigating
        global debugWait
        global path
        global nextPath
#        r.sleep()

        
        #goal assumed reached when navigateToGoal Returns.         
        
        wayPoints = self.getWayPointListFromStringCommand(path)
        
        #Call Rudis Functions with list for coordinates, called wayPoints. 
        #TODO: Should this function be a ROS action?        
            #simply sends coordinates to other node. This wakes up 
        self.navigateToGoal(wayPoints)
        
#        while doneNavigating == False:
#            rospy.loginfo('still navigating')            
#            rospy.sleep(5)
        
        #if goal is lineZone, change state to StateFreeAtLineZone           
        position = path
        status = mes_mobile_status()
        status.header.stamp = rospy.Time.now()
        status.state = mes_mobile_status.STATE_FREE
        status.position = position
        pubStatus.publish(status)
       
        if path == 'Line':
            path = nextPath
            return 'outcome1'            
        #else, change to StateFreeAtCoordinateZone
        else:
            path = nextPath
            return 'outcome2'


    def navigateToGoal(self,waypoints):
        global path        
        global coordNavClient
        global debugWait
            
        goal = MoveBaseGoal()        #self.getWayPointListFromStringCommand(path)
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = waypoints
        
        # Fill in the goal here
        coordNavClient.send_goal(goal)
        #wait 30 secs for robot to navigate to goal.
        coordNavClient.wait_for_result(rospy.Duration.from_sec(SERVER_WAIT_TIME_COORD))
        
        #add communication delay for everything to sync.        
        rospy.sleep(2)      

    def getWayPointListFromStringCommand(self,path):
        #It might be necessary to know the current position and which zone,
        #to know what waypoints to use. But firstly, it will be a single waypoint goal.
#        if path == 'Line':
#            return [(0,3)]
#        else if path == 'Dispenser':
#            return [(-5,2)]
        
        name2coord = {}
        
        pos = Point(4.971,-0.712,0.000)
        ori = Quaternion(0.000,0.000,0.705,0.709)
        pose = Pose(pos,ori)
        name2coord['Dispenser'] = pose

        pos = Point(4.428,0.100,0.000)
        ori = Quaternion(0.000,0.000,0.000,1.000)
        pose = Pose(pos,ori)
        name2coord['InBox'] = pose
        
        pos = Point(4.428,0.700,0.000)
        ori = Quaternion(0.000,0.000,0.707,0.707)
        pose = Pose(pos,ori)
        name2coord['Station1'] = pose
        
        pos = Point(4.928,0.700,0.000)
        ori = Quaternion(0.000,0.000,0.707,0.707)
        pose = Pose(pos,ori)
        name2coord['Station2'] = pose
        
        pos = Point(5.428,0.700,0.000)
        ori = Quaternion(0.000,0.000,0.707,0.707)
        pose = Pose(pos,ori)        
        name2coord['Station3'] = pose

        pos = Point(1.920,-0.450,0.000)
        ori = Quaternion(0.000,0.000,1.000,0.000)
        pose = Pose(pos,ori)                
        name2coord['RampOut'] = pose

        pos = Point(1.920,0.100,0.000)
        ori = Quaternion(0.000,0.000,0.000,1.000)
        pose = Pose(pos,ori)        
        name2coord['RampIn'] = pose

        pos = Point(0.000,-1.000,0.000)
        ori = Quaternion(0.000,0.000,1.000,0.000)
        pose = Pose(pos,ori)        
        name2coord['FloorOut'] = pose

        pos = Point(0.000,1.000,0.000)
        ori = Quaternion(0.000,0.000,0.000,1.000)
        pose = Pose(pos,ori)        
        name2coord['FloorIn'] = pose

        pos = Point(-0.500,-2.5000,0.000)
        ori = Quaternion(0.000,0.000,-0.707,-0.707)
        pose = Pose(pos,ori)        
        name2coord['Line'] = pose

#        posi = Point(1,2,0)
#        ori = Quaternion(3,4,5,6)
#        pose = Pose(pos,ori)        
#        LoadOff1
#
#        posi = Point(1,2,0)
#        ori = Quaternion(3,4,5,6)
#        pose = Pose(pos,ori)        
#        LoadOn1
#
#        posi = Point(1,2,0)
#        ori = Quaternion(3,4,5,6)
#        pose = Pose(pos,ori)        
#        LoadOff2
#
#        posi = Point(1,2,0)
#        ori = Quaternion(3,4,5,6)
#        pose = Pose(pos,ori)                
#        LoadOn2
#
#        posi = Point(1,2,0)
#        ori = Quaternion(3,4,5,6)
#        pose = Pose(pos,ori)        
#        LoadOff3
#
#        posi = Point(1,2,0)
#        ori = Quaternion(3,4,5,6)
#        pose = Pose(pos,ori)        
#        LoadOn3
        #not a list, only a single Pose object
        return name2coord[path]

        

        
# define state StateNavigateInLineZone
# Call Isaacs Line following action, that automatically finds and stops at a 
# certain QR code(or end of line). Then, 
class StateNavigateInLineZone(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
        self.atStartOfLine = True
        
        

    def execute(self, userdata):
        global position
        global path
        global nextPath
        global r
        global doneNavigating
        global debugWait
#        r.sleep()
        global cameFromLoadOff
        global latestRobotCell

        if self.atStartOfLine == True:
            rospy.loginfo("navigateToLongStretch")
            self.navigateToLongStretch()
            self.atStartOfLine = False

        if path == 'FloorIn':
            rospy.loginfo("navigateToFloorIn")

            self.navigateToFloorIn()
            self.atStartOfLine = True
        else:
            #rospy.loginfo("navigateToQR")
            if cameFromLoadOff == False:
                QRId = self.getQRId()
                latestRobotCell = QRId
                #TODO: QRId for endOfLine = 0!!! Maybe???
                self.navigateToQR(QRId)
               
               
                if self.loadOnOrOff() == "LoadOn":
                    self.followLineToCross()
                    cameFromLoadOff = False #not necessary
                else:
                     cameFromLoadOff = True
            else:
                self.followLineToCross()
                
                    
            #if to 
            self.turn90DegRight()
            
        
            
        
        #TODO: What if QR not found? Try again or error?
        
#        while doneNavigating == False:
#            rospy.sleep(5)
            
        position = path
        status = mes_mobile_status()
        status.header.stamp = rospy.Time.now()
        status.state = mes_mobile_status.STATE_WORKING
        status.position = position 
        pubStatus.publish(status)
       
        ##################################################################
        #TODO: Handle that navigation should continue in coordinate or load zone.
        #even without coming from a free state
        #DONE in coordinatezone, because path is set to FloorIn, so robot will navigate to center of this zone.
        #########################################################################
        
        #if at end of line, go to state StateNavigateInCoordinateZone, with way point = centerOfFloorInZone
        if path == 'FloorIn':
            
            #change to StateNavigateInCoordinateZone
            return 'outcome1'

        else:
            
            #in order to back up to LoadOff
            #TODO: should be left() when LoadOn!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            #Change to StateNavigateInLoadZone              
            return 'outcome2'

    
          
    def getQRId(self):
        global path
        #TODO: Get actual format soon!
#        if path == 'LoadOn0':
#            return 0
#        else if path == 'LoadOff1':
#            return 1
#       
        QR = "ERROR"
        if path == "LoadOff3" or path == "LoadOn3":
            QR = "Robot 3"
        
        if path == "LoadOff2" or path == "LoadOn2":
            QR = "Robot 2"

        if path == "LoadOff1" or path == "LoadOn1":
            QR = "Robot 1"
        
        return QR

    def navigateToLongStretch(self):
        rospy.loginfo("navigating to LongStretch")        
        self.followLineToCross()
        self.turn90DegRight()
        self.followLineToCross()
        self.turn90DegRight()
      

    def navigateToQR(self,QRId):
        rospy.loginfo("navigating to QR")
        self.followLineToQR(QRId)
        return 0
        
    def navigateToFloorIn(self):
        rospy.loginfo("navigating to FloorIn")
        #QRId = 'Robot 1'
        global cameFromLoadOff
        
        goal = "End " + latestRobotCell
#        rospy.loginfo("goal:", goal)
        
        self.followLineToQR(goal)
        
        
        
        self.turn90DegRight()
        self.followLineToCross()        
        self.turn90DegRight()
        self.followLineToCross()
        cameFromLoadOff = False
        

    def followLineOut(self):
        rospy.loginfo("following line out")
        global goCellClient
            
        goal = GocellGoal()
        goal.cell_name = 'out'#'LoanOn1'
        
        # Fill in the goal here
        goCellClient.send_goal(goal)
        goCellClient.wait_for_result(rospy.Duration.from_sec(SERVER_WAIT_TIME))
        rospy.sleep(2)
        
    def followLineToCross(self):
        rospy.loginfo("following line to cross")
        global goCellClient
            
        goal = GocellGoal()
        goal.cell_name = ''#'LoanOn1'
        
        # Fill in the goal here
        goCellClient.send_goal(goal)
        goCellClient.wait_for_result(rospy.Duration.from_sec(SERVER_WAIT_TIME))
        rospy.sleep(2)


    def followLineToQR(self,QRId):
        rospy.loginfo("following line to QR")
        global goCellClient
        
        goal = GocellGoal()
        goal.cell_name = QRId#'Robot 1'#'LoanOn1'
        
        # Fill in the goal here
        goCellClient.send_goal(goal)
        goCellClient.wait_for_result(rospy.Duration.from_sec(SERVER_WAIT_TIME))
        rospy.sleep(2)

    def followLineToEnd(self):
        rospy.loginfo("following line to end")
        global goCellClient
        
        #TODO: Determine end signal...
        goal = GocellGoal()
        
        goal.cell_name = 'End '#'LoanOn1'
        
        # Fill in the goal here
        goCellClient.send_goal(goal)
        goCellClient.wait_for_result(rospy.Duration.from_sec(SERVER_WAIT_TIME))
        rospy.sleep(2)

    def turn90DegRight(self):
        rospy.loginfo("turning 90 deg right")
        global goCellClient
        
        goal = spin_degreesGoal()
        goal.direction = 'right'
        
        # Fill in the goal here
        spin90Client.send_goal(goal)
        spin90Client.wait_for_result(rospy.Duration.from_sec(SERVER_WAIT_TIME))
        rospy.sleep(2)
        
        
    def turn90DegLeft(self):
        rospy.loginfo("turning 90 deg left")
        global goCellClient
        
        goal = spin_degreesGoal()
        goal.direction = 'left'
        
        # Fill in the goal here
        spin90Client.send_goal(goal)
        spin90Client.wait_for_result(rospy.Duration.from_sec(SERVER_WAIT_TIME))
        rospy.sleep(2)
        
        return 0
        
    def loadOnOrOff(self):
        if(path == 'LoadOn1' or path == 'LoadOn2' or path == 'LoadOn3'):
            return 'LoadOn'
        else:
            return 'LoadOff'
            
            
            
            
            
# define state StateNavigateInLoadZone
# Dependent on destination zone, back up different amounts, or move forward to line. Only tip in LoadOn scenario 
class StateNavigateInLoadZone(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
        #if already loaded, navigate out of cell and 
       
                
    def execute(self, userdata):
        global position
        global doneNavigating
        global r
        global debugWait
        global path
        global nextPath
        global cameFromLoadOff
#        r.sleep()
        
       
       
        if path == 'Line':
            #TODO: Is it really Independent of whether in LoadOn or LoadOff zone?
            self.driveForwardToLine()
        else:
            goal = self.loadOnOrOff()
            if goal == 'LoadOn':
                self.driveBackwardsToLoadOn()
                cameFromLoadOff = False
                #change to StateFreeAtLoadZone                
#                return 'outcome2' 
            else:
                self.driveBackwardsToLoadOff()
                cameFromLoadOff = True
                #change to StateFreeAtLoadZone
#                return 'outcome2'
           
           
        rospy.sleep(debugWait)  
#        while doneNavigating == False:
#            rospy.sleep(1)
#        doneNavigating = False
            ###############################################
            #TODO: Handle how to wait for tip. 
                #Maybe don't wait at all...            
            ###################################################
        
        #Change to StateFreMeAtLine
        #TODO: OBS: Normally, StateFreeAtLineZone assumes that the robot is
        # at the start of the line zone....
        position = path #here it is "Line"
        status = mes_mobile_status()  
        status.header.stamp = rospy.Time.now()           
        status.state = mes_mobile_status.STATE_FREE
        status.position = position 
        pubStatus.publish(status)     
        
        if path == 'Line':
            path = nextPath
            return 'outcome1'
        #else, determine if in LoadOn or LoadOff zone
        else:
            path = nextPath
            return 'outcome2'
    
    def loadOnOrOff(self):
        if(path == 'LoadOn1' or path == 'LoadOn2' or path == 'LoadOn3'):
            return 'LoadOn'
        else:
            return 'LoadOff'
 
 
    def followLineOut(self):
        rospy.loginfo("following line out")
        global goCellClient
            
        goal = GocellGoal()
        goal.cell_name = 'out'#'LoanOn1'
        
        # Fill in the goal here
        goCellClient.send_goal(goal)
        goCellClient.wait_for_result(rospy.Duration.from_sec(SERVER_WAIT_TIME))
        rospy.sleep(2)
            
    def driveForwardToLine(self):
        rospy.loginfo("driving forward to line")
        global cameFromLoadOff
        #TODO: Ask Rudi how to set a certain measurement moovement.
#        setMotorsForward()        
#        if atLoadOnZone():
#            if dist > 0.4 && lineVisible():
#                turnLeft90Deg()
#        else:
#            if dist > 0.7 && lineVisible():
#                turnLeft90Deg()
        if cameFromLoadOff == True:
            self.followLineToCross()
        else:
            self.followLineOut()
        self.turn90DegLeft()
        
        return 0       

    def driveBackwardsToLoadOff(self):
        rospy.loginfo("driving backwards to LoadOff")
        global goCellBackClient
        global latestRobotCell
            
        goal = GocellGoalBackwards()

        #Send robot name.        
        goal.cell_name = latestRobotCell#self.getQRId()#'out'#'LoanOn1'
        
        # Fill in the goal here
        goCellBackClient.send_goal(goal)
        goCellBackClient.wait_for_result(rospy.Duration.from_sec(SERVER_WAIT_TIME))
        rospy.sleep(2)
#        setMotorsReverse()        
#        if dist > 0.4:
        return 0
        
    def driveBackwardsToLoadOn(self):
        rospy.loginfo("driving backwards to LoadOn")
#        setMotorsReverse()        
#        if dist > 0.7:
        global goCellBackClient
            
        goal = GocellGoalBackwards()

        #Send robot name.        
        goal.cell_name = ""#getQRId()#'out'#'LoanOn1'
        
        # Fill in the goal here
        goCellBackClient.send_goal(goal)
        goCellBackClient.wait_for_result(rospy.Duration.from_sec(SERVER_WAIT_TIME))
        rospy.sleep(2)
#        setMotorsReverse()        
#        if dist > 0.4:
        return 0


   
        
    def followLineToCross(self):
        rospy.loginfo("following line to cross")
        global goCellClient
            
        goal = GocellGoal()
        goal.cell_name = ''#'LoanOn1'
        
        # Fill in the goal here
        goCellClient.send_goal(goal)
        goCellClient.wait_for_result(rospy.Duration.from_sec(SERVER_WAIT_TIME))
        rospy.sleep(2)

    def turn90DegLeft(self):
        rospy.loginfo("turning 90 deg left")
        global goCellClient
        
        goal = spin_degreesGoal()
        goal.direction = 'left'
        
        # Fill in the goal here
        spin90Client.send_goal(goal)
        spin90Client.wait_for_result(rospy.Duration.from_sec(SERVER_WAIT_TIME))
        rospy.sleep(2)
        
        return 0


    def getQRId(self):
        global path
        #TODO: Get actual format soon!
#        if path == 'LoadOn0':
#            return 0
#        else if path == 'LoadOff1':
#            return 1
#       
        QR = "ERROR"
        if path == "LoadOff3" or path == "LoadOn3":
            QR = "Robot 3"
        
        if path == "LoadOff2" or path == "LoadOn2":
            QR = "Robot 2"

        if path == "LoadOff1" or path == "LoadOn1":
            QR = "Robot 1"
        
        return QR    
# define state Bar
#class Bar(smach.State):
#    def __init__(self):
#        smach.State.__init__(self, outcomes=['outcome2'])
#
#    def execute(self, userdata):
#        rospy.loginfo('Executing state BAR')
#        return 'outcome2'
        


def mes_mobile_command_callback(data):
    #rospy.loginfo('In mes_mobile_command_callback()')    
    #rospy.loginfo(data)
    global path
    global nextPath    
    
    nextPath = data.path
    rospy.loginfo("nextPath: " + nextPath)
    global MESCommand    
    MESCommand = data.command
    rospy.loginfo("MESCommand: " + MESCommand)


def done_navigating_callback(data):
    rospy.loginfo('In done_navigating_callback()')    
    rospy.loginfo(data)
    global doneNavigating    
    doneNavigating = data

# main
def main():
    global r
    
    rospy.init_node('decisionLayer')
    r = rospy.Rate(0.5)

    rospy.loginfo("desicionLayer started")

#TODO: subscribe to and use messages from MES server
    #Subscribe to MES Server client topic. Receive command and path
    rospy.Subscriber("mes_mobile_command", mes_mobile_command, mes_mobile_command_callback)
    rospy.Subscriber("done_navigating",Bool,done_navigating_callback)

   


    # spin() simply keeps python from exiting until this node is stopped
   # rospy.spin()
    
    
    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container

#STATE_FREE_AT_COORDINATE_ZONE
#STATE_FREE_AT_LINE_ZONE
#STATE_FREE_AT_LOAD_ZONE
#STATE_NAVIGATE_IN_COORDINATE_ZONE
#STATE_NAVIGATE_IN_LINE_ZONE
#STATE_NAVIGATE_IN_LOAD_ZONE


    with sm_top:
        # Add states to the container
        smach.StateMachine.add('STATE_FREE_AT_COORDINATE_ZONE', StateFreeAtCoordinateZone(), 
                               transitions={'outcome1':'STATE_FREE_AT_COORDINATE_ZONE', #obsolete
                                            'outcome2':'STATE_NAVIGATE_IN_COORDINATE_ZONE'})
        smach.StateMachine.add('STATE_FREE_AT_LINE_ZONE', StateFreeAtLineZone(), 
                               transitions={'outcome1':'STATE_FREE_AT_LINE_ZONE', 
                                            'outcome2':'STATE_NAVIGATE_IN_LINE_ZONE'})
        smach.StateMachine.add('STATE_FREE_AT_LOAD_ZONE', StateFreeAtLoadZone(), 
                               transitions={'outcome1':'STATE_FREE_AT_LOAD_ZONE', 
                                            'outcome2':'STATE_NAVIGATE_IN_LOAD_ZONE'})
        smach.StateMachine.add('STATE_NAVIGATE_IN_COORDINATE_ZONE', StateNavigateInCoordinateZone(), 
                               transitions={'outcome1':'STATE_FREE_AT_LINE_ZONE', 
                                            'outcome2':'STATE_FREE_AT_COORDINATE_ZONE',
                                            'outcome3':'STATE_NAVIGATE_IN_COORDINATE_ZONE'})
                                            #'outcome2':'outcome4'})
        smach.StateMachine.add('STATE_NAVIGATE_IN_LINE_ZONE', StateNavigateInLineZone(), 
                               transitions={'outcome1':'STATE_NAVIGATE_IN_COORDINATE_ZONE', 
                                            'outcome2':'STATE_NAVIGATE_IN_LOAD_ZONE'})
        smach.StateMachine.add('STATE_NAVIGATE_IN_LOAD_ZONE', StateNavigateInLoadZone(), 
                               transitions={'outcome1':'STATE_FREE_AT_LINE_ZONE', 
                                            'outcome2':'STATE_FREE_AT_LOAD_ZONE'})                                            
        
#        sm_line_navigation = smach.StateMachine(outcomes=['outcome5'])
#
#        with sm_line_navigation:
#            ###########################################################            
#            #TODO: Line navigation is only a problem of turning at appripriate times, with known map of line.
#            # this is not a state machine, but more a 
#            ############################################################3
#            smach.StateMachine.add('STATE_NAVIGATE_TO_CROSS', StateNavigateToCross(), 
#                                   transitions={'outcome1':'BAR', 
#                                                'outcome2':'outcome4'})
#            smach.StateMachine.add('STATE_FOLLOW_LINE', StateFollowLine(), 
#                                   transitions={'outcome1':'BAR', 
#                                                'outcome2':'outcome4'})
#            smach.StateMachine.add('STATE_NAVIGATE_TO_CROSS', StateNavigateToCross(), 
#                                   transitions={'outcome1':'BAR', 
#                                                'outcome2':'outcome4'})
    # Execute SMACH plan
    sis = smach_ros.IntrospectionServer('decisionLayerIntrospectionServer', sm_top, '/SM_ROOT')
    sis.start()
    
        
    outcome = sm_top.execute()  


if __name__ == '__main__':
    main()
