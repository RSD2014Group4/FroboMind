#!/usr/bin/env python

import rospy
import actionlib
import tf

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from pose_to_estimate.msg import PoseEstimateAction, PoseEstimateGoal, PoseEstimateResult

class PoseFromMarker():
    def __init__(self):
        self.initTopics()
        self.initAction()
        return
        
    def initTopics(self):
        self.marker_pose = Odometry()
        self.marker_pose_received = False
        self.marker_pose_topic = rospy.get_param("~marker_pose_topic",'/fmInformation/marker_locator_pose')
        rospy.Subscriber(self.marker_pose_topic, Odometry, self.on_marker_topic)

        self.estimate_pose_topic = rospy.get_param("~estimate_pose_topic",'/initialpose')
        self.estimate_pose_pub = rospy.Publisher(self.estimate_pose_topic, PoseWithCovarianceStamped, queue_size=10)

    def initAction(self):
        self.action_name = rospy.get_param("~action_name",'posefrommarker')
        self.action_server = actionlib.SimpleActionServer(self.action_name, PoseEstimateAction, execute_cb=self.execute_cb, auto_start = False)
        self.action_result = PoseEstimateResult()
        self.action_server.start()

    def on_marker_topic(self, msg):
        self.marker_pose = msg
        self.marker_pose_received = True
        
    def poseMessage(self):
        newframe = "map"
        posemsg = PoseWithCovarianceStamped()
        posemsg.pose = tf.TransformerROS.transformPose(newframe,self.marker_pose.pose)
        posemsg.header = self.marker_pose.header
        posemsg.header.frame_id = newframe
        return posemsg
        
    def actionReturnError(self):
        self.action_result.poseSent = False
        self.action_server.set_aborted(self.action_result)
        rospy.loginfo("Action failed")
     
    def actionReturnSucceeded(self):
        self.action_result.poseSent = True
        self.action_server.set_succeeded(self.action_result)
        rospy.loginfo("Action succeeded")

    def execute_cb(self, goal):
        self.action_result.poseSent = False

        if not self.marker_pose_received:
            rospy.logerr("No pose received")
            self.actionReturnError()
            return
        
        rospy.loginfo("Trying to send pose")
        if goal.sendPose:
            try:
                pose = self.poseMessage()
                self.estimate_pose_pub.publish(pose)
                self.actionReturnSucceeded()
            except:
                rospy.logerr("Could not publish pose")
                self.actionReturnError()
        else:
            rospy.loginfo("No pose requested?")
            self.actionReturnError()

if __name__ == '__main__':
    rospy.init_node('pose_from_marker')
    try:
        node_class = PoseFromMarker()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
