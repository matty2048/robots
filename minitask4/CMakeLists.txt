cmake_minimum_required(VERSION 2.8.3)
project(minitask4)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS rospy std_msgs geometry_msgs)

catkin_install_python(PROGRAMS
  scripts/minitask4.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)