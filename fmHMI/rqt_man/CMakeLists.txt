cmake_minimum_required(VERSION 2.8.3)
project(rqt_man)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS msgs roscpp rospy std_msgs geometry_msgs message_generation)

catkin_python_setup()

add_message_files(
   FILES
   gui_command.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs# Or other packages containing msgs
   geometry_msgs
 )

catkin_package(
DEPENDS message_runtime std_msgs
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rqt_man
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

