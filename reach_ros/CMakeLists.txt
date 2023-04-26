cmake_minimum_required(VERSION 3.5)
project(reach_ros)

add_compile_options(-std=c++14)

find_package(reach REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
#find_package(eigen_conversions REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

#find_package(tf2_eigen REQUIRED)
#find_package(pluginlib REQUIRED)
#find_package(reach_core REQUIRED)
#find_package(reach_msgs REQUIRED)
#find_package(geometric_shapes REQUIRED)
##find_package(pcl_conversions REQUIRED)
#find_package(PCL REQUIRED)

# ######################################################################################################################
# BUILD ##
# ######################################################################################################################
set(INCLUDE_DIRS include ${reach_INCLUDE_DIRS}) #${PCL_INCLUDE_DIRS}
include_directories(${INCLUDE_DIRS})

# Plugins
add_library(
  ${PROJECT_NAME}_plugins
  src/utils.cpp
  # Evaluator
  src/evaluation/manipulability_moveit.cpp
  src/evaluation/joint_penalty_moveit.cpp
  src/evaluation/distance_penalty_moveit.cpp
  # IK Solver
  src/ik/moveit_ik_solver.cpp
  # Display
  src/display/ros_display.cpp)
target_link_libraries(${PROJECT_NAME}_plugins ${catkin_LIBRARIES} reach::reach)

ament_target_dependencies(${PROJECT_NAME}_plugins
  reach
  ament_cmake
  ament_cmake_ros
  #eigen_conversions
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  sensor_msgs
  visualization_msgs)


# Reach study node
add_executable(${PROJECT_NAME}_node src/reach_study_node.cpp)
target_link_libraries(
  ${PROJECT_NAME}_node
  yaml-cpp
  reach
  ament_cmake
  ament_cmake_ros
  #eigen_conversions
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  sensor_msgs
  visualization_msgs)

# Demo
add_subdirectory(demo)

# ######################################################################################################################
# INSTALL ##
# ######################################################################################################################

install(
  TARGETS ${PROJECT_NAME}_plugins ${PROJECT_NAME}_node
  ARCHIVE DESTINATION  ${PROJECT_NAME}
  LIBRARY DESTINATION  lib
  RUNTIME DESTINATION  include)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})

ament_package()
