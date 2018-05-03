cmake_minimum_required(VERSION 2.8.3)
project(reach_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_msgs
  sensor_msgs
)

add_message_files(
  FILES
    ReachRecord.msg
    ReachDatabase.msg
)

add_service_files(
  FILES
    SampleMesh.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    moveit_msgs
    sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    moveit_msgs
    sensor_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

#############
## Testing ##
#############
