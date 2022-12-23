cmake_minimum_required(VERSION 3.0.2)
project(paws_description)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
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