cmake_minimum_required(VERSION 2.8.3)
project(reach_core)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  interactive_markers
  pcl_ros
  pluginlib
  reach_msgs
  tf
  tf_conversions
  visualization_msgs
)

find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    reach_study_utils
    reach_database
    ik_helper
    reach_visualizer
    reach_study
    plugins
  CATKIN_DEPENDS
    eigen_conversions
    geometry_msgs
    interactive_markers
    pcl_ros
    pluginlib
    reach_msgs
    tf
    tf_conversions
    visualization_msgs
)

###########
## BUILD ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Utilites Library
add_library(reach_study_utils
  src/utils/general_utils.cpp
  src/utils/visualization_utils.cpp
)
target_link_libraries(reach_study_utils
  ${catkin_LIBRARIES}
)
add_dependencies(reach_study_utils
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Reach Database Library
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

# IK Helper Library
add_library(ik_helper
  src/core/ik_helper.cpp
)
target_link_libraries(ik_helper
  ${catkin_LIBRARIES}
  reach_database
)
add_dependencies(ik_helper
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Reach Visualizer Library
add_library(reach_visualizer
  src/core/reach_visualizer.cpp
)
target_link_libraries(reach_visualizer
  ${catkin_LIBRARIES}
  ik_helper
)
add_dependencies(reach_visualizer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Reach Study Library
add_library(reach_study
  src/core/reach_study.cpp
)
target_link_libraries(reach_study
  ${catkin_LIBRARIES}
  reach_visualizer
)
add_dependencies(reach_study
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Plugins Library
add_library(plugins
  src/plugins/impl/multiplicative_factory.cpp
)
target_link_libraries(plugins
  ${catkin_LIBRARIES}
)

# Reach Study Node
add_executable(robot_reach_study_node
  src/robot_reach_study_node.cpp
)
target_link_libraries(robot_reach_study_node
  ${catkin_LIBRARIES}
  reach_study
)
add_dependencies(robot_reach_study_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
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
add_dependencies(data_loader
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#############
## INSTALL ##
#############

install(
  TARGETS
    reach_study_utils
    reach_database
    ik_helper
    reach_visualizer
    reach_study
    plugins
    robot_reach_study_node
    sample_mesh_server
    data_loader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
