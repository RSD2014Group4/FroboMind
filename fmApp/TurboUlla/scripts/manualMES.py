#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import sys


import rospy
import smach
import smach_ros

import math
import numpy

from TurboUlla.msg import mes_mobile_status, mes_mobile_command


def main():
    
    
    rospy.init_node('dummy')

    r = rospy.Rate(0.1) # 10hz
    

    pub = rospy.Publisher("mes_mobile_command", mes_mobile_command, queue_size = 10)
    
    rospy.sleep(1)
    rospy.loginfo("now awake!")
    msg = mes_mobile_command()
        
    msg.command = msg.COMMAND_NAVIGATE#NAVIGATE#TIP
    msg.path = sys.argv[1]
#    rospy.loginfo('Executing state StateNavigateInCoordinateZone')
#    rospy.loginfo(sys.argv[1])
    
    pub.publish(msg)    
    
    rospy.spin()
    
#    while(1):
#        msg = mes_mobile_command()
#        
#        msg.command = msg.COMMAND_NAVIGATE
#        msg.path = sys.argv[1]
#    #    rospy.loginfo('Executing state StateNavigateInCoordinateZone')
#    #    rospy.loginfo(sys.argv[1])
#        
#        pub.publish(msg)
#        r.sleep()

if __name__ == '__main__':
    main()