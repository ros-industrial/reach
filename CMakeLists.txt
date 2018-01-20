cmake_minimum_required(VERSION 2.8.3)
project(robot_reach_study)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  interactive_markers
  message_generation
  message_runtime
  moveit_core
  moveit_msgs
  moveit_ros_planning
  pcl_ros
  sensor_msgs
  tf
  tf_conversions
  visualization_msgs
)

find_package(cmake_modules REQUIRED)

find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

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
    moveit_msgs
    sensor_msgs
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    reach_database
    ik_helper
    reach_visualizer
    reach_study
    reach_study_utils
  CATKIN_DEPENDS
    moveit_core
    moveit_msgs
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Utilites Library
add_library(reach_study_utils
  src/utils/database_utils.cpp
  src/utils/general_utils.cpp
  src/utils/kinematics_utils.cpp
  src/utils/visualization_utils.cpp
)
target_link_libraries(reach_study_utils
  ${catkin_LIBRARIES}
)
add_dependencies(reach_study_utils
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Reach Database Library
add_library(reach_database
  src/core/reach_database.cpp
)
target_link_libraries(reach_database
  ${catkin_LIBRARIES}
  reach_study_utils
)
add_dependencies(reach_database
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## IK Helper Library
add_library(ik_helper
  src/core/ik_helper.cpp
)
target_link_libraries(ik_helper
  ${catkin_LIBRARIES}
  reach_database
)

## Reach Visualizer Library
add_library(reach_visualizer
  src/core/reach_visualizer.cpp
)
target_link_libraries(reach_visualizer
  ${catkin_LIBRARIES}
  ik_helper
)

## Reach Study Library
add_library(reach_study
  src/core/reach_study.cpp
)
target_link_libraries(reach_study
  ${catkin_LIBRARIES}
  reach_visualizer
)

## Reach Study Node
add_executable(robot_reach_study_node
  src/robot_reach_study_node.cpp
)
target_link_libraries(robot_reach_study_node
  ${catkin_LIBRARIES}
  reach_study
)

# Sample Mesh Server Node
add_executable(sample_mesh_server
  src/sample_mesh_server_node.cpp
)
target_link_libraries(sample_mesh_server
  ${catkin_LIBRARIES}
)
add_dependencies(sample_mesh_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Data Loader Node
add_executable(data_loader
  src/data_loader_node.cpp
)
target_link_libraries(data_loader
  ${catkin_LIBRARIES}
  reach_database
)
