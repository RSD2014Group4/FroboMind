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

from line_til_cross_action.msg import *
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
debugWait = 5

#set from callback function from MES_command topic
global MESCommand
MESCommand = mes_mobile_command.COMMAND_WAIT
#target position/zone
#path = 'FloorOut'
path = 'Station1'
#current position/zone
#position = 'FloorOut'
position = 'Station1'

#TODO: do enum type        
ON = 1
OFF = 0

global goCellClient
global spin90Client
goCellClient = actionlib.SimpleActionClient('Gocell', GocellAction)
spin90Client = actionlib.SimpleActionClient('spin_degrees', spin_degreesAction)

global coordNavClient 
coordNavClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)#Pose)



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
#        r.sleep()
        
        #If nothing new received from MES, stay free
        while MESCommand == mes_mobile_command.COMMAND_WAIT:
            #sleep for 5 seconds
            rospy.loginfo('command was wait. 5 sec')
            status = mes_mobile_status()
            status.header.stamp = rospy.Time.now()
            status.state = mes_mobile_status.STATE_FREE
            status.position = position 
            pubStatus.publish(status)
            rospy.sleep(debugWait)
 

        
        #If command received from MES server, Go to NavigateInCoordinateZone
        if MESCommand == mes_mobile_command.COMMAND_NAVIGATE:
            #TODO:Publish state change to OEE topic
            status = mes_mobile_status()
            status.header.stamp = rospy.Time.now()
            status.state = mes_mobile_status.STATE_WORKING
            status.position = position 
            pubStatus.publish(status)
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
#        r.sleep()
        
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
            self.tip() #TODO: CALL ACTION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            rospy.sleep(debugWait)
            status = mes_mobile_status()
            status.header.stamp = rospy.Time.now()
            status.state = mes_mobile_status.STATE_WORKING
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
            #Reset MESCommand to zero
                #whenever this state, "free", a status have been sent,
                #and a new command will be set by the callback function
            MESCommand = mes_mobile_command.COMMAND_WAIT
            return 'outcome2'  
#        else:
            #ERROR  



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
            return 'outcome1'            
        #else, change to StateFreeAtCoordinateZone
        else:
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
        coordNavClient.wait_for_result(rospy.Duration.from_sec(5.0))
        
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
        
        pos = Point(5.200,-0.650,0.000)
        ori = Quaternion(0.000,0.000,1.000,0.000)
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

        pos = Point(1,2,0)
        ori = Quaternion(3,4,5,6)
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

        global r
        global doneNavigating
        global debugWait
#        r.sleep()

        if self.atStartOfLine == True:
            self.navigateToLongStretch()
            self.atStartOfLine = False

        if path == 'FloorIn':
            self.navigateToFloorIn()
            self.atStartOfLine = True
        else:
            QRId = self.getQRId()        
            #TODO: QRId for endOfLine = 0!!! Maybe???
            self.navigateToQR(QRId)
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
        #TODO: Get actual format soon!
#        if path == 'LoadOn0':
#            return 0
#        else if path == 'LoadOff1':
#            return 1
#       
        return path

    def navigateToLongStretch(self):
        self.followLineToCross()
        self.turn90DegRight()
        self.followLineToCross()
        self.turn90DegRight()
      

    def navigateToQR(self,QRId):
        self.followLineToQR(QRId)
        return 0
        
    def navigateToFloorIn(self):
        QRId = 'LoadOn1'        
        self.followLineToQR(QRId)
        self.followLineToCross()
        self.turn90DegRight()
        self.followLineToCross()        
        self.turn90DegRight()
        self.followLineToEnd()        
        
    def followLineToCross(self):
        global goCellClient
            
        goal = GocellGoal()
        goal.cell_name = ''#'LoanOn1'
        
        # Fill in the goal here
        goCellClient.send_goal(goal)
        goCellClient.wait_for_result(rospy.Duration.from_sec(5.0))
        rospy.sleep(2)


    def followLineToQR(self,QRId):
        global goCellClient
        
        goal = GocellGoal()
        goal.cell_name = QRId#'Robot 1'#'LoanOn1'
        
        # Fill in the goal here
        goCellClient.send_goal(goal)
        goCellClient.wait_for_result(rospy.Duration.from_sec(5.0))
        rospy.sleep(2)

    def followLineToEnd(self):
        global goCellClient
        
        #TODO: Determine end signal...
        goal = GocellGoal()
        goal.cell_name = ''#'LoanOn1'
        
        # Fill in the goal here
        goCellClient.send_goal(goal)
        goCellClient.wait_for_result(rospy.Duration.from_sec(5.0))
        rospy.sleep(2)

    def turn90DegRight(self):
        global goCellClient
        
        goal = spin_degreesGoal()
        goal.direction = 'right'
        
        # Fill in the goal here
        spin90Client.send_goal(goal)
        spin90Client.wait_for_result(rospy.Duration.from_sec(5.0))
        rospy.sleep(2)
        
        
    def turn90DegLeft(self):
        global goCellClient
        
        goal = spin_degreesGoal()
        goal.direction = 'left'
        
        # Fill in the goal here
        spin90Client.send_goal(goal)
        spin90Client.wait_for_result(rospy.Duration.from_sec(5.0))
        rospy.sleep(2)
        
        
            
        return 0
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
#        r.sleep()
        
       
       
        if path == 'Line':
            #TODO: Is it really Independent of whether in LoadOn or LoadOff zone?
            self.driveForwardToLine()
        else:
            goal = self.loadOnOrOff()
            if goal == 'LoadOn':
                self.driveBackwardsToLoadOn()
                #change to StateFreeAtLoadZone                
#                return 'outcome2' 
            else:
                self.driveBackwardsToLoadOff()
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
            return 'outcome1'
        #else, determine if in LoadOn or LoadOff zone
        else:
            return 'outcome2'
    
    def loadOnOrOff(self):
        if(path == 'LoadOn1' or path == 'LoadOn2' or path == 'LoadOn3'):
            return 'LoadOn'
        else:
            return 'LoadOff'
             
    def driveForwardToLine(self):
        #TODO: Ask Rudi how to set a certain measurement moovement.
#        setMotorsForward()        
#        if atLoadOnZone():
#            if dist > 0.4 && lineVisible():
#                turnLeft90Deg()
#        else:
#            if dist > 0.7 && lineVisible():
#                turnLeft90Deg()
#        
        return 0       

    def driveBackwardsToLoadOff(self):
#        setMotorsReverse()        
#        if dist > 0.4:
            return 0
        
    def driveBackwardsToLoadOn(self):
#        setMotorsReverse()        
#        if dist > 0.7:
            return 0                   

    def tip(self):
#        activateTipper()
        return 0
    
# define state Bar
#class Bar(smach.State):
#    def __init__(self):
#        smach.State.__init__(self, outcomes=['outcome2'])
#
#    def execute(self, userdata):
#        rospy.loginfo('Executing state BAR')
#        return 'outcome2'
        


def mes_mobile_command_callback(data):
    rospy.loginfo('In mes_mobile_command_callback()')    
    rospy.loginfo(data)
    global path    
    path = data.path
    global MESCommand    
    MESCommand = data.command

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
