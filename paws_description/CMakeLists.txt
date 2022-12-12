cmake_minimum_required(VERSION 3.0.2)
project(paws_description)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
)
