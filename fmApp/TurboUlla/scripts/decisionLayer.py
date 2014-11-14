#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

import math
import numpy

from mes_mobile_status.msg import mes_mobile_status
from mes_mobile_command.msg import mes_mobile_command


MESCommand = mes_mobile_command.COMMAND_WAIT
#target position/zone
path = 'FloorOut'
#current position/zone
position = 'FloorOut'

#TODO: do enum type        
OFF = 0
ON = 1


#TODO: Dirty code, should not be global variable...
pub = rospy.Publisher("mes_mobile_status", mes_mobile_status, queue_size = 10)

        
# define state StateFreeAtCoordinateZone
class StateFreeAtCoordinateZone(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StateFreeAtCoordinateZone')
        
        
        #If nothing new received from MES, stay free
        if MESCommand == COMMAND_WAIT:
            return 'outcome1'
        
        #If command received from MES server, Go to NavigateInCoordinateZone
        if MESCommand == COMMAND_NAVIGATE:
            #TODO:Publish state change to OEE topic
            status = mes_mobile_status()
            status.state = mes_mobile_status.STATE_WORKING
            status.position = position 
            pub.publish()
            #Reset MESCommand to zero
                #whenever this state, "free", a status have been sent,
                #and a new command will be set by the callback function
            MESCommand = COMMAND_WAIT
            return 'outcome2' #loop this state  


# define state StateFreeAtLineZone
class StateFreeAtLineZone(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StateFreeAtLineZone')
        
        
        #If nothong received from MES, stay free
        if MESCommand == COMMAND_WAIT:
            return 'outcome1'
        
        #If command received from MES server, Go to NavigateInCoordinateZone
        if MESCommand == COMMAND_NAVIGATE:
            #TODO:Publish state change to OEE topic
            return 'outcome2' #loop this state  


# define state StateFreeAtLoadZone
class StateFreeAtLoadZone(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StateFreeALoadZone')
        
        
        #If nothong received from MES, stay free
        if MESCommand == COMMAND_WAIT:
            return 'outcome1'
        
        #If command received from MES server, Go to NavigateInCoordinateZone
        if MESCommand == COMMAND_NAVIGATE:
            #TODO:Publish state change to OEE topic
            return 'outcome2' #loop this state  



# define state StateNavigateInCoordinateZone
# Uses rabbit line following to navigate a list of internally defined waypoints,
# in order to reach a goal set by the MES server command path.
class StateNavigateInCoordinateZone(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])


    def execute(self, userdata):
        rospy.loginfo('Executing state StateNavigateInCoordinateZone')
        
        #adfadsfasdf
        wayPoints = self.getWayPointListFromStringCommand(path)
        
        #Call Rudis Functions with list for coordinates, called wayPoints. 
        #TODO: Should this function be a ROS action?        
        #TODO: Should the waypoints include ORIENTATION?
        self.navigateToGoal(wayPoints)
        
        #goal assumed reached when navigateToGoal Returns.         
        
        #if goal is lineZone, change state to StateFreeAtLineZone           
        if path == 'Line':
            return 'outcome1'
            #TODO: Send status to MES, line zone, working
        #else, change to StateFreeAtCoordinateZone
        else:
            #TODO: Send status to MES, free, coordinata zone
            return 'outcome2'
 
    def getWayPointListFromStringCommand(self,path):
        #It might be necessary to know the current position and which zone,
        #to know what waypoints to use. But firstly, it will be a single waypoint goal.
#        if path == 'Line':
#            return [(0,3)]
#        else if path == 'Dispenser':
#            return [(-5,2)]
        return 0

        
    def navigateToGoal(self,waypoints):
#        callRabbit()        
        return 0
        
# define state StateNavigateInLineZone
# Call Isaacs Line following action, that automatically finds and stops at a 
# certain QR code(or end of line). Then, 
class StateNavigateInLineZone(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StateNavigateInLineZone')
        
        QRId = self.getQRId()        
        #TODO: QRId for endOfLine = 0!!! Maybe???
        self.navigateToQR(QRId)        
        #TODO: What if QR not found? Try again or error?

        #if at end of line, go to state StateNavigateInCoordinateZone, with way point = centerOfFloorInZone
        if path == 'FloorIn':
            #change to StateNavigateInCoordinateZone
            return 'outcome1'

        else:
            self.turn90DegRight()
            #Change to StateNavigateInLoadZone              
            return 'outcome2'
            
    def getQRId(self):
#        if path == 'LoadOn0':
#            return 0
#        else if path == 'LoadOff1':
#            return 1
#        
        return 0

    def navigateToQR(self,id):
#        startLineFollowAction        
#        if QRFound == true:
            return 0
        
        
    def turn90DegRight(self):
        return 0
# define state StateNavigateInLoadZone
# Dependent on destination zone, back up different amounts, or move forward to line. Only tip in LoadOn scenario 
class StateNavigateInLoadZone(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StateNavigateInLoadZone')
        
        #if already loaded, navigate out of cell and 
        if path == 'Line':
            #TODO: Is it really Independent of whether in LoadOn or LoadOff zone?
            self.driveForwardToLine()
            #Change to StateFreeAtLine
            #TODO: OBS: Normally, StateFreeAtLineZone assumes that the robot is
            # at the start of the line zone....             
            return 'outcome1'
        #else, determine if in LoadOn or LoadOff zone
        else:
            goal = LoadOnOrOff()
            if goal == 'LoadOn':
                self.driveBackwardsToLoadOn()
                #change to StateFreeAtLoadZone                
                return 'outcome2' 
            else:
                self.driveBackwardsTOLoadOff()
                tip();                
                #change to StateFreeAtLoadZone
                return 'outcome2'
     
    def driveForwardToLine(self):
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
        


def callback(data):
    rospy.loginfo(data.data)
    path = data.path
    MESCommand = data.command

# main
def main():
    rospy.init_node('decisionLayer')

    rospy.loginfo("desicionLayer started")

#TODO: subscribe to and use messages from MES server
    #Subscribe to MES Server client topic. Receive command and path
    rospy.Subscriber("mes_mobile_command", mes_mobile_command, callback)
    

    # spin() simply keeps python from exiting until this node is stopped
   # rospy.spin()
    
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container

    with sm:
        # Add states to the container
        smach.StateMachine.add('STATE_FREE_AT_COORDINATE_ZONE', StateFreeAtCoordinateZone(), 
                               transitions={'outcome1':'STATE_FREE_AT_COORDINATE_ZONE', 
                                            'outcome2':'STATE_NAVIGATE_IN_COORDINATE_ZONE'})
        smach.StateMachine.add('STATE_FREE_AT_LINE_ZONE', StateFreeAtLineZone(), 
                               transitions={'outcome1':'STATE_FREE_AT_LINE_ZONE', 
                                            'outcome2':'STATE_NAVIGATE_IN_LINE_ZONE'})
        smach.StateMachine.add('STATE_FREE_AT_LOAD_ZONE', StateFreeAtLoadZone(), 
                               transitions={'outcome1':'STATE_FREE_AT_LOAD_ZONE', 
                                            'outcome2':'STATE_NAVIGATE_IN_LOAD_ZONE'})
        smach.StateMachine.add('STATE_NAVIGATE_IN_COORDINATE_ZONE', StateNavigateInCoordinateZone(), 
                               transitions={'outcome1':'STATE_FREE_AT_LINE_ZONE', 
                                            #'outcome2':'STATE_FREE_AT_COORDINATE_ZONE'})
                                            'outcome2':'outcome4'})
        smach.StateMachine.add('STATE_NAVIGATE_IN_LINE_ZONE', StateNavigateInLineZone(), 
                               transitions={'outcome1':'STATE_NAVIGATE_IN_COORDINATE_ZONE', 
                                            'outcome2':'STATE_NAVIGATE_IN_LOAD_ZONE'})
        smach.StateMachine.add('STATE_NAVIGATE_IN_LOAD_ZONE', StateNavigateInLoadZone(), 
                               transitions={'outcome1':'STATE_FREE_AT_LINE_ZONE', 
                                            'outcome2':'STATE_FREE_AT_LOAD_ZONE'})                                            

    # Execute SMACH plan
    outcome = sm.execute()  


if __name__ == '__main__':
    main()