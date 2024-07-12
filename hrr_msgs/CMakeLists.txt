cmake_minimum_required(VERSION 3.0.2)
project(hrr_msgs)

find_package(catkin
  REQUIRED COMPONENTS
    actionlib_msgs
    geometry_msgs
    message_generation
    message_runtime
    comau_msgs
)

add_message_files(
  DIRECTORY msg
)

add_service_files(
  DIRECTORY srv
)

add_action_files(
  DIRECTORY action
)

generate_messages(
   DEPENDENCIES
   actionlib_msgs
   geometry_msgs
   comau_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hrr_msgs
  CATKIN_DEPENDS
     actionlib_msgs
     geometry_msgs
     message_runtime
     comau_msgs
)

include_directories()
