cmake_minimum_required(VERSION 2.8.3)
project(reach_plugins)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
  xmlrpcpp
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_utils
    evaluation_plugins
    ik_solver_plugins
  CATKIN_DEPENDS
    pluginlib
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
