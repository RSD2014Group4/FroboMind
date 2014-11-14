cmake_minimum_required(VERSION 2.8.3)
project(waypoint_action_server)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs message_generation msgs rospy nav_msgs)

atkin_python_setup()

add_action_files(DIRECTORY action FILES waypoint_nav.action)

generate_messages(
  DEPENDENCIES 
  actionlib_msgs nav_msgs std_msgs
)

catkin_package()