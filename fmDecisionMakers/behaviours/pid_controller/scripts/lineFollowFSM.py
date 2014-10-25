#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

import math
import numpy

from pid_controller.msg import line_points
from geometry_msgs.msg import TwistStamped


lineData = [[3.0,2.0],[1.0,1.0]]

#class BehaviourFollowLine():



#Follow line using camera until Correct QR is found.
class StateFollowLineQR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        rospy.loginfo("in StateFollowLineQR.init")
        self.p = 1 #The further from the goal, the more power.
        self.i = 0.1 #If some external force is influencing the robot, I will slowly overpower this force
        self.d = -0.5 #Should counter the integrated part when an error of 0 is reoptained
        self.myCenter = [0,0]#[0,0] #Robot center located at (0,0)
        self.memCTError = 0        
        self.iCTError = 0
        self.PIDFreq =2.0
        self.PIDPeriod = 1/self.PIDFreq
        self.PIDRate = rospy.Rate(self.PIDFreq) # 10hz
        self.publisher = rospy.Publisher('/fmCommand/cmd_vel', TwistStamped)

        

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
        rospy.loginfo("in regto0")
        #TODO: Create ROS topic CONTROL_TWIST
        CTError = self.calcCTError(linePoints)
        dCTError = (CTError - self.memCTError)/(self.PIDPeriod)
        self.iCTError = self.iCTError + ((self.memCTError + CTError)/2)*(self.PIDPeriod)
        self.memCTError = CTError
        rospy.loginfo(CTError)
        rospy.loginfo(self.iCTError)
        rospy.loginfo(dCTError)
        rospy.loginfo(linePoints[0][0])
        #rospy.loginfo("p: " + CTError + ", i: " + self.iCTError + ", d: " + dCTError)

        controlSignal = self.p*CTError + self.i*self.iCTError + self.d*dCTError
        controlSignalTwist = controlSignal #!!!!!!!!!!!!!!!!HOW TO SEND TWIST?

        twist = TwistStamped()
        twist.linear.x = 0.2;                   # our forward speed
        twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!        
        twist.angular.x = 0; twist.angular.y = 0;   #          or these!
        twist.angular.z = controlSignal;    


        self.publisher.publish(twist)        

        return controlSignalTwist
    
    def compareQRCode(self):
        #TODO: subscribe to QR topic        
        return 0
    
    def execute(self, userdata):
        rospy.loginfo('Executing state StateFollowLineQR')
        #TODO: Get rsd_LineData from Isaacs code        
        
        #linePoints1 = [[1,3],[3,4]]       
        
        self.regulateErrorTo0(lineData)
        QRCodeCorrect = self.compareQRCode()
        self.PIDRate.sleep() #!!!!!!!!!!!!!!!how does this work with SMACH
        if QRCodeCorrect == 1:
            return 'outcome1'
        else:
            return 'outcome2'

     
        
 # define state Bar
class StateWaitAtQR(smach.State):
    def __init__(self): 
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StateWaitAtQR')
        return 'outcome2' #loop this state  



# define state Bar
#class Bar(smach.State):
#    def __init__(self):
#        smach.State.__init__(self, outcomes=['outcome2'])
#
#    def execute(self, userdata):
#        rospy.loginfo('Executing state BAR')
#        return 'outcome2'
        


def callback(data):
    #rospy.loginfo(data.data)
    #rospy.loginfo("%f y: %f" % (data.x1, data.y1))
    global lineData
    lineData = [[data.x1,data.y1],[data.x2,data.y2]]
   # rospy.loginfo(data.x1)
# main
def main():
    #rospy.init_node('smach_example_state_machine')

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('lineFollowFSM', anonymous=True)

    rospy.loginfo("Started")

    #publisher = rospy.Publisher('cmd_vel', Twist)
    rospy.Subscriber("rsd_camera/line_points", line_points, callback)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    
    
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
