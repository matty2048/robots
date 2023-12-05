cmake_minimum_required(VERSION 2.8.3)
project(minitask5)
find_package(catkin REQUIRED COMPONENTS 
   rospy
   std_msgs
   geometry_msgs
   message_generation
)

add_message_files(
  FILES
  controller.msg
  image_proc.msg
  object_data.msg
  frontiers.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 LIBRARIES minitask5
 CATKIN_DEPENDS rospy std_msgs geometry_msgs message_runtime
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
  scripts/frontier_finder.py
  scripts/image_proc.py
  scripts/object_avoid.py
  scripts/move_to.py
  scripts/localisation.py
  scripts/mapping.py
  scripts/frontier_mapping.py
  # scripts/scan_obj.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)