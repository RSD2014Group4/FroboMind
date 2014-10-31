cmake_minimum_required(VERSION 2.8.3)
project(amcl_localizer)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf geometry_msgs amcl gmapping)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

