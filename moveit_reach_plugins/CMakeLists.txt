cmake_minimum_required(VERSION 2.8.3)
project(moveit_reach_plugins)

add_compile_options(-std=c++14)

find_package(reach_core REQUIRED)

find_package(
  catkin REQUIRED
  COMPONENTS eigen_conversions
             interactive_markers
             moveit_core
             moveit_msgs
             moveit_ros_planning_interface
             visualization_msgs)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}_utils
  ${PROJECT_NAME}_plugins
  CATKIN_DEPENDS
  eigen_conversions
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  visualization_msgs)

# ######################################################################################################################
# BUILD ##
# ######################################################################################################################

include_directories(include ${catkin_INCLUDE_DIRS})

# Utils Library
add_library(${PROJECT_NAME}_utils src/utils.cpp)
add_dependencies(${PROJECT_NAME}_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_utils ${catkin_LIBRARIES} reach::reach_core)

# Plugins
add_library(${PROJECT_NAME}_plugins
  # Evaluator
  src/evaluation/manipulability_moveit.cpp
  src/evaluation/joint_penalty_moveit.cpp
  src/evaluation/distance_penalty_moveit.cpp
  # IK Solver
  src/ik/moveit_ik_solver.cpp
  # Display
  src/display/moveit_reach_display.cpp
)
target_link_libraries(${PROJECT_NAME}_plugins ${catkin_LIBRARIES} ${PROJECT_NAME}_utils reach::reach_core)

# Reach study node
add_executable(${PROJECT_NAME}_node src/reach_study_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} yaml-cpp reach::reach_core)

# Demo
add_subdirectory(demo)

# ######################################################################################################################
# INSTALL ##
# ######################################################################################################################

install(
  TARGETS ${PROJECT_NAME}_utils
          ${PROJECT_NAME}_plugins
          ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
