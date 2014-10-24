# -*- coding: utf-8 -*-
"""
Spyder Editor

This temporary script file is located here:
/home/boerresen/.spyder2/.temp.py
"""

#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

import math
import numpy





#class BehaviourFollowLine():


#Follow line using camera until Correct QR is found.
class StateFollowLineQR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.p = 1 #The further from the goal, the more power.
        self.i = 0.1 #If some external force is influencing the robot, I will slowly overpower this force
        self.d = -0.5 #Should counter the integrated part when an error of 0 is reoptained
        self.myCenter = [0,0]#[0,0] #Robot center located at (0,0)

    def calcCTError(self,linePoints):
        #TODO:Handle changing lines or corners        
        
        #Use line knowledge, ax+b, form Isaac, given in robot coordinate system
        #Calculate orthogonal distance from robot center to line. 
            #Dist from point to line can be calculated by basic math formula
            #REF:http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
            #REF image:http://mathworld.wolfram.com/images/equations/Point-LineDistance2-Dimensional/NumberedEquation10.gif

        pointOnLine1 = linePoints[0]
        pointOnLine2 = linePoints[1]

        num = numpy.linalg.norm( numpy.linalg.det( [ numpy.subtract( pointOnLine2,pointOnLine1 ), numpy.subtract( pointOnLine2,self.myCenter ) ]  ) )
        denum = ( numpy.linalg.norm( numpy.subtract(pointOnLine2,pointOnLine1) ) )
        dist =  num/denum
        return dist
    
    def regulateErrorTo0(self,linePoints):
        #TODO: Create ROS topic CONTROL_TWIST
        CTError = self.calcCTError(linePoints)
        controlSignal = self.p*CTError + self.i*CTError + self.d*CTError
        controlSignalTwist = controlSignal
        return controlSignalTwist
    
    def compareQRCode(self):
        return 0
    
    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        #TODO: Get rsd_LineData from Isaacs code        
        
        linePoints1 = [[1,3],[3,4]]        
        self.regulateErrorTo0(linePoints1)
        QRCodeCorrect = self.compareQRCode()
        if QRCodeCorrect == 1:
            return 'outcome1'
        else:
            return 'outcome2'

     
        
 # define state Bar
class StateWaitAtQR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('State wait QR')
        return 'outcome2'   



# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome2'
        

def callback():
	rospy.loginfo("Inside call_back")

# main
def main():
    rospy.init_node('smach_example_state_machine')

	
   # rospy.Subscriber("rsd_camera/line_points",msg,callback)


    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('STATE_FOLLOW_LINE_QR', StateFollowLineQR(), 
                               transitions={'outcome1':'STATE_WAIT_AT_QR', 
                                            'outcome2':'STATE_FOLLOW_LINE_QR'})
        smach.StateMachine.add('STATE_WAIT_AT_QR', StateWaitAtQR(), 
                               transitions={'outcome2':'STATE_WAIT_AT_QR'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
