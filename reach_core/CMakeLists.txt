cmake_minimum_required(VERSION 3.5)
project(reach_core)

add_compile_options(-std=c++14)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_core REQUIRED)
# find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(reach_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  interactive_markers
  moveit_core
  # pcl_ros
  pcl_conversions
  pluginlib
  rclcpp
  reach_msgs
  tf2_eigen
  tf2_ros
  visualization_msgs
)

ament_export_include_directories(include)

ament_export_libraries(
  src/${PROJECT_NAME}
  src/${PROJECT_NAME}_plugins
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# catkin_package(
#   INCLUDE_DIRS
#     include
#   LIBRARIES
#     ${PROJECT_NAME}
#     ${PROJECT_NAME}_plugins
#   CATKIN_DEPENDS
#     eigen_conversions
#     geometry_msgs
#     interactive_markers
#     pcl_ros
#     pluginlib
#     reach_msgs
#     tf2_ros
#     tf2_eigen
#     visualization_msgs
# )

###########
## BUILD ##
###########

include_directories(
  include
)
include_directories(${PCL_INCLUDE_DIRS})

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
# target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_plugins
# )
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Plugins Library
add_library(${PROJECT_NAME}_plugins
  src/plugins/impl/multiplicative_factory.cpp
)
# target_link_libraries(${PROJECT_NAME}_plugins
# )
ament_target_dependencies(${PROJECT_NAME}_plugins ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Reach Study Node
add_executable(robot_reach_study_node
  src/robot_reach_study_node.cpp
)
target_link_libraries(robot_reach_study_node
  ${PROJECT_NAME}
)
ament_target_dependencies(robot_reach_study_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# Load Point Cloud Server Node
add_executable(load_point_cloud_server_node
  src/load_point_cloud_server_node.cpp
)
target_link_libraries(load_point_cloud_server_node ${PROJECT_NAME}
)
ament_target_dependencies(load_point_cloud_server_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Data Loader Node
add_executable(data_loader
  src/data_loader_node.cpp
)
target_link_libraries(data_loader
  ${PROJECT_NAME}
)
ament_target_dependencies(data_loader
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

##########
## TEST ##
##########

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(${PROJECT_NAME}_plugin_utest test/plugin.test test/plugin_utest.cpp)
  target_link_libraries(${PROJECT_NAME}_plugin_utest ${PROJECT_NAME})
endif()

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
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)
