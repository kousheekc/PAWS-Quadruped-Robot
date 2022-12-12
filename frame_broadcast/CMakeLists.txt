cmake_minimum_required(VERSION 3.0.2)
project(frame_broadcast)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  tf2
  tf2_ros
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/frame_broadcast.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)