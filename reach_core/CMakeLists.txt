cmake_minimum_required(VERSION 2.8.3)
project(reach_core)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS roscpp pluginlib)

find_package(Boost REQUIRED COMPONENTS serialization program_options)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS io search)
find_package(yaml-cpp REQUIRED)

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
    roscpp
    pluginlib)

# ######################################################################################################################
# BUILD ##
# ######################################################################################################################

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Reach Study Library
add_library(
  ${PROJECT_NAME}
  src/utils.cpp
  src/reach_database.cpp
  src/reach_visualizer.cpp
  src/reach_study.cpp
  src/reach_study_comparison.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen Boost::serialization ${PCL_LIBRARIES} yaml-cpp)

# Plugins Library
add_library(${PROJECT_NAME}_plugins
  src/plugins/multiplicative_factory.cpp
  src/plugins/point_cloud_target_pose_generator.cpp
  src/plugins/logger.cpp
  src/plugins/no_op.cpp)
target_link_libraries(${PROJECT_NAME}_plugins ${catkin_LIBRARIES} Eigen3::Eigen ${PCL_LIBRARIES})

# Reach Study Node
add_executable(${PROJECT_NAME}_node src/app/reach_study_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})

# Data Loader Node
add_executable(${PROJECT_NAME}_data_loader src/app/data_loader_node.cpp)
target_link_libraries(${PROJECT_NAME}_data_loader ${catkin_LIBRARIES} ${PROJECT_NAME})

# ######################################################################################################################
# TEST ##
# ######################################################################################################################

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(${PROJECT_NAME}_plugin_utest test/plugin.test test/plugin_utest.cpp)
  target_link_libraries(${PROJECT_NAME}_plugin_utest ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

# ######################################################################################################################
# INSTALL ##
# ######################################################################################################################

install(
  TARGETS ${PROJECT_NAME}
          ${PROJECT_NAME}_plugins
          ${PROJECT_NAME}_node
          ${PROJECT_NAME}_data_loader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
