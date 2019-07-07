cmake_minimum_required(VERSION 2.8.3)
project(moveit_reach_plugins)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  pluginlib
  reach_msgs
  visualization_msgs
  xmlrpcpp
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_utils
    evaluation_plugins
    ik_solver_plugins
    reach_display_plugins
  CATKIN_DEPENDS
    eigen_conversions
    interactive_markers
    moveit_core
    moveit_msgs
    moveit_ros_planning_interface
    reach_msgs
    pluginlib
    visualization_msgs
    xmlrpcpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#Utils Library
add_library(${PROJECT_NAME}_utils
  src/utils.cpp
)
add_dependencies(${PROJECT_NAME}_utils
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_utils
  ${catkin_LIBRARIES}
)

# Evaluation Plugins
add_library(evaluation_plugins
  src/evaluation/manipulability_moveit.cpp
  src/evaluation/joint_penalty_moveit.cpp
  src/evaluation/distance_penalty_moveit.cpp
  src/evaluation/multiplicative_factory.cpp
)
add_dependencies(evaluation_plugins
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(evaluation_plugins
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
)

# MoveIt IK Solver Plugin
add_library(ik_solver_plugins
  src/ik/moveit_ik_solver.cpp
  src/ik/discretized_moveit_ik_solver.cpp
)
add_dependencies(ik_solver_plugins
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ik_solver_plugins
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
)

# MoveIt Reach Display Plugin
add_library(reach_display_plugins
  src/display/moveit_reach_display.cpp
)
add_dependencies(reach_display_plugins
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(reach_display_plugins
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils
)

# IK Plugin Test
add_executable(ik_plugin_test
  test/plugin_test_node.cpp
)
add_dependencies(ik_plugin_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ik_plugin_test
  ${catkin_LIBRARIES}
)
