cmake_minimum_required(VERSION 3.0.2)
project(paws_kinematics)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/inverse_kinematics.py
  src/whole_body_control.py
  src/dummy_foot_pose_pub.py
  src/dummy_body_pose_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)