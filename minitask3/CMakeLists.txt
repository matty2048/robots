cmake_minimum_required(VERSION 2.8.3)
project(minitask3)
find_package(catkin REQUIRED COMPONENTS 
   rospy
   std_msgs
   geometry_msgs
   message_generation
)

add_message_files(
  FILES
  state_mt3.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 LIBRARIES minitask3
 CATKIN_DEPENDS rospy std_msgs geometry_msgs message_runtime
 DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
#   Scripts/green_obstacle.py
   Scripts/green_walk.py
   Scripts/random_turn.py
   Scripts/random_walk.py
   Scripts/script.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
