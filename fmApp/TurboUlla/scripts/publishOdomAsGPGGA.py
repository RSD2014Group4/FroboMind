#!/bin/python

# This script is supported by "klemraekker" and duct tape :)
# Quick hack to publish GPGGA messages from the stage simulator in order to test the frobomind kalman filter


# FROM : Type: nav_msgs/Odometry
# TO : msgs/gpgga_tranmerc

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from msgs.msg import gpgga_tranmerc

def onMsg(msg):
    pubMsg = gpgga_tranmerc()
    pubMsg.header = msg.header
    pubMsg.time = str(msg.header.stamp)
    pubMsg.northing = msg.pose.pose.position.y
    pubMsg.easting = msg.pose.pose.position.x
    pubMsg.fix = np.uint8(4)
    pubMsg.sat = np.uint8(6)
    pubMsg.hdop = np.double(1.0)
    pub.publish(pubMsg)


if __name__ == '__main__':
    pub_topic = "/fmInformation/gpgga_tranmerc"
    sub_topic = "/base_pose_ground_truth"
    rospy.init_node('publishOdomAsGPGGA')
    pub = rospy.Publisher(pub_topic, gpgga_tranmerc, queue_size=10)
    sub = rospy.Subscriber(sub_topic, Odometry, onMsg )
    rospy.spin()
	