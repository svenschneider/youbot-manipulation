cmake_minimum_required(VERSION 2.8.3)
project(youbot_arm_kinematics_moveit)

find_package(catkin REQUIRED
  COMPONENTS
    pluginlib
    moveit_core
    urdf
    tf_conversions
    youbot_arm_kinematics
)

catkin_package()

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

add_library(youbot_arm_kinematics_moveit
  ros/src/youbot_arm_kinematics_moveit.cpp
  ros/src/kinematics_logger.cpp
)

target_link_libraries(youbot_arm_kinematics_moveit
  ${catkin_LIBRARIES}
)

install(TARGETS youbot_arm_kinematics_moveit
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_add_gtest(load_moveit_plugin_test
  ros/test/load_moveit_plugin_test.cpp
)

target_link_libraries(load_moveit_plugin_test
  ${catkin_LIBRARIES}
)

catkin_add_gtest(youbot_arm_kinematics_moveit_test
  ros/test/youbot_arm_kinematics_moveit_test.cpp
)

target_link_libraries(youbot_arm_kinematics_moveit_test
  ${catkin_LIBRARIES}
)

catkin_add_gtest(configuration_comparator_test
  ros/test/configuration_comparator_test.cpp
)