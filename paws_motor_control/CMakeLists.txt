cmake_minimum_required(VERSION 3.0.2)
project(paws_motor_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/motor_control_gazebo.py
  src/motor_control_rviz.py
  src/dummy_joint_angle_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)