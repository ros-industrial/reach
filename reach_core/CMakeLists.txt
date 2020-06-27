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
  tf2_ros
  tf2_eigen
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
    ${PROJECT_NAME}
    ${PROJECT_NAME}_plugins
  CATKIN_DEPENDS
    eigen_conversions
    geometry_msgs
    interactive_markers
    pcl_ros
    pluginlib
    reach_msgs
    tf2_ros
    tf2_eigen
    visualization_msgs
)

###########
## BUILD ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Reach Study Library
add_library(${PROJECT_NAME}
  # Utilities
  src/utils/general_utils.cpp
  src/utils/visualization_utils.cpp
  # Tools
  src/core/reach_database.cpp
  src/core/ik_helper.cpp
  src/core/reach_visualizer.cpp
  # Reach Study
  src/core/reach_study.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Plugins Library
add_library(${PROJECT_NAME}_plugins
  src/plugins/impl/multiplicative_factory.cpp
)
target_link_libraries(${PROJECT_NAME}_plugins
  ${catkin_LIBRARIES}
)

# Reach Study Node
add_executable(robot_reach_study_node
  src/robot_reach_study_node.cpp
)
target_link_libraries(robot_reach_study_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
add_dependencies(robot_reach_study_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Load Point Cloud Server Node
add_executable(load_point_cloud_server_node
  src/load_point_cloud_server_node.cpp
)
target_link_libraries(load_point_cloud_server_node
  ${catkin_LIBRARIES}
)
add_dependencies(load_point_cloud_server_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Data Loader Node
add_executable(data_loader
  src/data_loader_node.cpp
)
target_link_libraries(data_loader
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
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
    ${PROJECT_NAME}
    ${PROJECT_NAME}_plugins
    robot_reach_study_node
    load_point_cloud_server_node
    data_loader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
