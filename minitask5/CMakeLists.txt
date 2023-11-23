cmake_minimum_required(VERSION 2.8.3)
project(minitask5)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs)

add_message_files(
 FILES
 controller.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS 
   rospy
   std_msgs
   geometry_msgs
   message_generation
)


catkin_package(
 LIBRARIES minitask5
 CATKIN_DEPENDS rospy std_msgs geometry_msgs message_runtime
 DEPENDS system_lib
)

# Potentially need to move
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/main.py
  scripts/goal_finder.py
  scripts/image_proc.py
  scripts/object_avoid.py
  scripts/search.py
  scripts/localisation.py
  scripts/mapping.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
