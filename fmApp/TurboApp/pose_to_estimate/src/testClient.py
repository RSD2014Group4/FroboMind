#!/usr/bin/env python

import rospy
import actionlib

from pose_to_estimate.msg import PoseEstimateAction, PoseEstimateGoal, PoseEstimateResult

def testClient():
    client = actionlib.SimpleActionClient('posefrommarker', PoseEstimateAction)
    client.wait_for_server()
    
    # Creates a goal to send to the action server.
    goal = PoseEstimateGoal()
    goal.sendPose = True
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('test_pose_from_marker_action')
        result = testClient()
        print "Result:"
        print result
    except rospy.ROSInterruptException:
        pass