cmake_minimum_required(VERSION 3.5)
project(reach_ros)

add_compile_options(-std=c++17)

find_package(reach REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(boost_plugin_loader REQUIRED)
find_package(eigen_conversions REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ######################################################################################################################
# BUILD ##
# ######################################################################################################################
set(INCLUDE_DIRS include)
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
  src/display/ros_display.cpp
)

target_include_directories(${PROJECT_NAME}_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_plugins PUBLIC
  ament_cmake
  boost_plugin_loader
  eigen_conversions
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
  visualization_msgs)

target_link_libraries(${PROJECT_NAME}_plugins PRIVATE reach::reach)

# Reach study node
add_executable(${PROJECT_NAME}_node src/reach_study_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node PUBLIC
  yaml-cpp
  ament_cmake
  boost_plugin_loader
  eigen_conversions
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  sensor_msgs
  visualization_msgs)
target_link_libraries(${PROJECT_NAME}_node PRIVATE reach::reach PUBLIC ${PROJECT_NAME}_plugins)

# Demo
#add_subdirectory(demo)

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
