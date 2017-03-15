cmake_minimum_required(VERSION 2.8.3)
project(robot_reach_study)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  message_runtime
  moveit_core
  moveit_msgs
  moveit_ros_planning
  pcl_ros
  visualization_msgs
  interactive_markers
  tf
  tf_conversions
  sensor_msgs
  vtk_viewer
  tool_path_planner
)

find_package(cmake_modules REQUIRED)
find_package(OpenMP)

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
 )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    moveit_core
    moveit_msgs
    sensor_msgs
)

add_compile_options(-std=c++11)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(include
  ${catkin_INCLUDE_DIRS})

add_executable(robot_reach_study src/reach_study.cpp
                                 src/reach_database.cpp
                                 src/interactive_ik.cpp
                                 src/ik_helper.cpp)

add_executable(sample_mesh_server src/sample_mesh_server.cpp)

add_executable(data_loader src/data_loader.cpp
                           src/reach_database.cpp)

add_dependencies(robot_reach_study ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
 robot_reach_study_generate_messages_cpp)

add_dependencies(sample_mesh_server sample_mesh_server_generate_messages_cpp)

target_link_libraries(robot_reach_study
  ${catkin_LIBRARIES})

target_link_libraries(sample_mesh_server
  ${catkin_LIBRARIES})

target_link_libraries(data_loader
  ${catkin_LIBRARIES})
