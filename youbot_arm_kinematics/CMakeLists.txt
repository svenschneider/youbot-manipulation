cmake_minimum_required(VERSION 2.8.3)
project(youbot_arm_kinematics)

find_package(catkin REQUIRED)
find_package(orocos_kdl REQUIRED)

catkin_package(
  INCLUDE_DIRS common/include
  LIBRARIES youbot_arm_kinematics
  DEPENDS
    orocos_kdl
)

include_directories(
  common/include
  ${orocos_kdl_INCLUDE_DIRS}
)

add_library(youbot_arm_kinematics
  common/src/inverse_kinematics.cpp
  common/src/logger.cpp
)

target_link_libraries(youbot_arm_kinematics
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

install(TARGETS youbot_arm_kinematics
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
