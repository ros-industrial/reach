cmake_minimum_required(VERSION 3.5)
project(moveit_reach_plugins)

add_compile_options(-std=c++11)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(reach_core REQUIRED)
find_package(reach_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
#find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)


###########
## BUILD ##
###########

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${reach_core_INCLUDE_DIRS}
)

# Utils Library
add_library(${PROJECT_NAME}_utils
  src/utils.cpp
)
target_include_directories(${PROJECT_NAME}_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_utils
  geometric_shapes
  moveit_msgs
  reach_msgs
  tf2_eigen
)

# Evaluation Plugins
add_library(evaluation_plugins
  src/evaluation/manipulability_moveit.cpp
  src/evaluation/joint_penalty_moveit.cpp
  src/evaluation/distance_penalty_moveit.cpp
)
target_include_directories(evaluation_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(evaluation_plugins
  ${PROJECT_NAME}_utils
  ${PCL_LIBRARIES}
)
ament_target_dependencies(evaluation_plugins
  geometric_shapes
  moveit_msgs
  reach_msgs
  tf2_eigen
  moveit_ros_planning_interface
  moveit_core
  pluginlib
)
pluginlib_export_plugin_description_file(reach_core eval_plugin_description.xml)

# MoveIt IK Solver Plugin
add_library(ik_solver_plugins
  src/ik/moveit_ik_solver.cpp
  src/ik/discretized_moveit_ik_solver.cpp
  src/ik/cartesian_retrieval_ik_solver.cpp
)
target_include_directories(ik_solver_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(ik_solver_plugins
  ${PROJECT_NAME}_utils
)
ament_target_dependencies(ik_solver_plugins
  moveit_ros_planning_interface
  geometric_shapes
#  reach_core
  pluginlib
  rclcpp
  tf2_eigen
)
pluginlib_export_plugin_description_file(reach_core ik_plugin_description.xml)
target_compile_definitions(ik_solver_plugins PRIVATE "MOVEIT_REACH_PLUGINS_BUILDING_LIBRARY")

# MoveIt Reach Display Plugin
add_library(reach_display_plugins
  src/display/moveit_reach_display.cpp
)
target_include_directories(reach_display_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(reach_display_plugins
  ${PROJECT_NAME}_utils
  ${PCL_LIBRARIES}
  ik_solver_plugins
)
pluginlib_export_plugin_description_file(reach_core display_plugin_description.xml)

# IK Plugin Test
#add_executable(ik_plugin_test
#  test/plugin_test_node.cpp
#)
#add_dependencies(ik_plugin_test
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS}
#)
#target_link_libraries(ik_plugin_test
#  ${catkin_LIBRARIES}
#)

#############
## INSTALL ##
#############

install(
  TARGETS
    ${PROJECT_NAME}_utils
    evaluation_plugins
    ik_solver_plugins
    reach_display_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_libraries(
  ${PROJECT_NAME}_utils
  evaluation_plugins
  ik_solver_plugins
  reach_display_plugins
)

ament_export_include_directories(
  include
)
ament_package()