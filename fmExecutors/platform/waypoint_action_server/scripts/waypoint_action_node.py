#!/usr/bin/env python

# Inspired by Waypoint navigation node by Kjeld Jensen <kjeld@frobomind.org>

import rospy
from math import pi, atan2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from waypoint_navigation import waypoint_navigation

class WptActionNode():
    def __init__(self):
        self.initParameters()
        self.initTopics()

        
    def initParameters(self):
        # get topic names
        self.pose_topic = rospy.get_param("~pose_sub",'/fmKnowledge/pose')
        self.cmdvel_topic = rospy.get_param("~cmd_vel_pub",'/fmCommand/cmd_vel')
        self.wptnav_status_topic = rospy.get_param("~status_pub",'/fmInformation/wptnav_status')
        self.pid_topic = rospy.get_param("~pid_pub",'/fmInformation/wptnav_pid')
        
        self.debug = rospy.get_param("~print_debug_information", 'true') 
        self.update_rate = rospy.get_param("~update_rate", 20)
        self.r = rospy.Rate(self.update_rate)
        
        # Waypoint nav specific
        self.w_dist = rospy.get_param("/diff_steer_wheel_distance", 0.2) # [m]
        self.drive_kp = rospy.get_param("~drive_kp", 1.0)
        self.drive_ki = rospy.get_param("~drive_ki", 0.0)
        self.drive_kd = rospy.get_param("~drive_kd", 0.0)
        self.drive_ff = rospy.get_param("~drive_feed_forward", 0.0)
        self.drive_max_output = rospy.get_param("~drive_max_output", 0.3)
        self.turn_kp = rospy.get_param("~turn_kp", 1.0)
        self.turn_ki = rospy.get_param("~turn_ki", 0.0)
        self.turn_kd = rospy.get_param("~turn_kd", 0.2)
        self.turn_ff = rospy.get_param("~turn_feed_forward", 0.0)
        self.turn_max_output = rospy.get_param("~turn_max_output", 0.5)
        
        self.max_linear_vel = rospy.get_param("~max_linear_velocity", 0.4)
        self.max_angular_vel = rospy.get_param("~max_angular_velocity", 0.4)

        self.wpt_def_tolerance = rospy.get_param("~wpt_default_tolerance", 0.5)
        self.wpt_def_drive_vel = rospy.get_param("~wpt_default_drive_velocity", 0.5)
        self.wpt_def_turn_vel = rospy.get_param("~wpt_default_turn_velocity", 0.3)
        self.wpt_def_wait_after_arrival = rospy.get_param("~wpt_default_wait_after_arrival", 0.0)
        self.wpt_def_implement = rospy.get_param("~wpt_default_implement_command", 0.0)

        self.target_ahead = rospy.get_param("~target_ahead", 1.0)
        self.turn_start_at_heading_err = rospy.get_param("~turn_start_at_heading_err", 20.0)
        self.turn_stop_at_heading_err = rospy.get_param("~turn_stop_at_heading_err", 2.0)
        self.ramp_drive_vel_at_dist = rospy.get_param("~ramp_drive_velocity_at_distance", 1.0)
        self.ramp_min_drive_vel = rospy.get_param("~ramp_min_drive_velocity", 0.1)
        self.ramp_turn_vel_at_angle = rospy.get_param("~ramp_turn_velocity_at_angle", 25.0)
        self.ramp_min_turn_vel = rospy.get_param("~ramp_min_turn_velocity", 0.05)
        self.stop_nav_at_dist = rospy.get_param("~stop_navigating_at_distance", 0.1)
    
    def initTopics(self):
        rospy.Subscriber(self.pose_topic, Odometry, self.on_pose_message)  
        self.cmd_vel_pub = rospy.Publisher(self.cmdvel_topic, TwistStamped)

    def initWaypointNav(self):
        self.wptnav = waypoint_navigation(self.update_rate, self.w_dist, self.drive_kp, self.drive_ki, self.drive_kd, self.drive_ff, self.drive_max_output, self.turn_kp, self.turn_ki, self.turn_kd, self.turn_ff, self.turn_max_output, self.max_linear_vel, self.max_angular_vel, self.wpt_def_tolerance, self.wpt_def_drive_vel, self.wpt_def_turn_vel, self.target_ahead, self.turn_start_at_heading_err, self.turn_stop_at_heading_err, self.ramp_drive_vel_at_dist, self.ramp_min_drive_vel, self.ramp_turn_vel_at_angle, self.ramp_min_turn_vel, self.stop_nav_at_dist, self.debug)        
        
    def on_pose_message(self, msg):
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = atan2(2*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        self.wptnav.state_update (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, msg.twist.twist.linear.x)

    def publish_cmd_vel_message(self):
        self.twist.header.stamp = rospy.Time.now()
        self.twist.twist.linear.x = self.linear_vel
        self.twist.twist.angular.z = self.angular_vel		
        self.cmd_vel_pub.publish (self.twist)

    def updater(self):
        while not rospy.is_shutdown():
            self.publish_cmd_vel_message()
            self.r.sleep()

if __name__ == '__main__':
    rospy.init_node('wptnav_action_node')

    try:
        node_class = WptActionNode()
    except rospy.ROSInterruptException:
        pass
