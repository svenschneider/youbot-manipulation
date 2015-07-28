cmake_minimum_required(VERSION 2.8.3)
project(youbot_gripper)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
