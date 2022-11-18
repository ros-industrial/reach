cmake_minimum_required(VERSION 3.10.0)

# Extract package name and version from package.xml
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(Boost REQUIRED COMPONENTS serialization program_options)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS io search)
find_package(yaml-cpp REQUIRED)
find_package(boost_plugin_loader REQUIRED)

find_package(OpenMP)
if(OPENMP_FOUND)
  if(NOT TARGET OpenMP::OpenMP_CXX)
    find_package(Threads REQUIRED)
    add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
    set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
    # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
    set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)

  endif()
endif()

# Reach Study Library
add_library(
  ${PROJECT_NAME} SHARED
  src/utils.cpp
  src/plugin_utils.cpp
  src/reach_database.cpp
  src/reach_visualizer.cpp
  src/reach_study.cpp
  src/reach_study_comparison.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
         Boost::serialization
         ${PCL_LIBRARIES}
         yaml-cpp
         boost_plugin_loader::boost_plugin_loader
         OpenMP::OpenMP_CXX)
target_compile_definitions(${PROJECT_NAME} PUBLIC SEARCH_LIBRARIES_ENV="REACH_PLUGINS" PLUGIN_LIBRARIES="${PROJECT_NAME}_plugins:reach_ros_plugins")
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 14)

# Plugins Library
add_library(
  ${PROJECT_NAME}_plugins SHARED
  src/plugins/multiplicative_evaluator.cpp
  src/plugins/point_cloud_target_pose_generator.cpp
  src/plugins/console_logger.cpp
  src/plugins/boost_progress_console_logger.cpp
  src/plugins/no_op.cpp)
target_link_libraries(
  ${PROJECT_NAME}_plugins
  PUBLIC ${PROJECT_NAME}
         Eigen3::Eigen
         ${PCL_LIBRARIES}
         boost_plugin_loader::boost_plugin_loader)
target_cxx_version(${PROJECT_NAME}_plugins PUBLIC VERSION 14)

# Reach Study Node
add_executable(${PROJECT_NAME}_app src/app/reach_study_node.cpp)
target_link_libraries(${PROJECT_NAME}_app PRIVATE ${PROJECT_NAME} Boost::program_options)
target_cxx_version(${PROJECT_NAME}_app PUBLIC VERSION 14)

# Data Loader Node
add_executable(${PROJECT_NAME}_data_loader src/app/data_loader_node.cpp)
target_link_libraries(${PROJECT_NAME}_data_loader PRIVATE ${PROJECT_NAME} Boost::program_options)
target_cxx_version(${PROJECT_NAME}_data_loader PUBLIC VERSION 14)

# ######################################################################################################################
# TEST ##
# ######################################################################################################################

if(ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# ######################################################################################################################
# INSTALL ##
# ######################################################################################################################

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# Package configuration
configure_package(
  NAMESPACE reach
  DEPENDENCIES
    "Boost REQUIRED COMPONENTS serialization program_options"
    "PCL REQUIRED COMPONENTS io search"
    Eigen3
    yaml-cpp
    boost_plugin_loader
    OpenMP
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugins ${PROJECT_NAME}_app ${PROJECT_NAME}_data_loader)
