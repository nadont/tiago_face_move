cmake_minimum_required(VERSION 3.0.2)
project(tiago_face_move)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  pal_detection_msgs
  move_base_msgs
  actionlib
  tf
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge pal_detection_msgs move_base_msgs actionlib tf
)

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install scripts
install(PROGRAMS
  src/scripts/face_detector.py
  src/scripts/face_follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

