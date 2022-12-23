cmake_minimum_required(VERSION 3.0.2)
project(paws_kinematics)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  tf2
  tf2_ros
  message_generation
)

add_message_files(
  FILES
  Phase.msg
  Joints.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/inverse_kinematics.py
  src/whole_body_control.py
  src/dummy_foot_pose_pub.py
  src/dummy_body_pose_pub.py
  src/dummy_phase_pub.py
  src/paws_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)