cmake_minimum_required(VERSION 2.8.3)
project(kalman_convention_fix)

find_package(catkin REQUIRED COMPONENTS
  roscpp tf
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(kalman_convention_fix_node src/kalman_convention_fix_node.cpp)


target_link_libraries(kalman_convention_fix_node
  ${catkin_LIBRARIES}
)
