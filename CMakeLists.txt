cmake_minimum_required(VERSION 2.8.3)
project(ros_cozmo)

find_package(catkin REQUIRED COMPONENTS actionlib message_generation)

add_action_files(
  DIRECTORY action
  FILES
  SayText.action
)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

install(PROGRAMS
  scripts/ros_cozmo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
