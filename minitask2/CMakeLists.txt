cmake_minimum_required(VERSION 2.8.3)
project(minitask2)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS rospy std_msgs geometry_msgs)

catkin_install_python(PROGRAMS
   random_walk.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


catkin_install_python(PROGRAMS
   obstacle_avoidance.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


catkin_install_python(PROGRAMS
   wall_walk.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)