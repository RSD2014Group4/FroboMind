#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

import math
import numpy
#import gometry_msgs
import std_msgs

class StateFreeAtCoordinateZone(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        rospy.sleep(5)
        return 'outcome1' 

# define state StateFreeAtLineZone
class StateFreeAtLineZone(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        rospy.sleep(5)
        return 'outcome1'
        
        #If nothing new received from MES, stay free
def main():
    global r
    
    rospy.init_node('decisionLayer')

    rospy.loginfo("desicionLayer started")



    # spin() simply keeps python from exiting until this node is stopped
   # rospy.spin()
    
    
    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container

    with sm_top:
        # Add states to the container
        smach.StateMachine.add('STATE_FREE_AT_COORDINATE_ZONE', StateFreeAtCoordinateZone(), 
                               transitions={'outcome1':'STATE_FREE_AT_LINE_ZONE', #obsolete
                                            'outcome2':'STATE_FREE_AT_LINE_ZONE'})
        smach.StateMachine.add('STATE_FREE_AT_LINE_ZONE', StateFreeAtLineZone(), 
                               transitions={'outcome1':'STATE_FREE_AT_COORDINATE_ZONE', 
                                            'outcome2':'STATE_FREE_AT_COORDINATE_ZONE'})

    # Execute SMACH plan
    sis = smach_ros.IntrospectionServer('decisionLayerIntrospectionServer', sm_top, '/SM_ROOT')
    sis.start()
    
        
    outcome = sm_top.execute()  


    

if __name__ == '__main__':
    main()