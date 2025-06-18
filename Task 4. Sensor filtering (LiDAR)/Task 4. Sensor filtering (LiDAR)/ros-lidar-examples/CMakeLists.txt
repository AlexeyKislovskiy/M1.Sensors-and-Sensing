cmake_minimum_required(VERSION 3.10)
project(ros_lidar_examples VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED
  rospy
  sensor_msgs
)

catkin_package()
