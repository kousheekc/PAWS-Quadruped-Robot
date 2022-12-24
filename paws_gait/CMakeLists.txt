cmake_minimum_required(VERSION 3.0.2)
project(paws_gait)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/foot_trajectory_generation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
