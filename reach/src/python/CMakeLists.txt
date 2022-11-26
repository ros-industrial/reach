# Add Python libraries
Python_add_library(
  ${PROJECT_NAME}_python
  MODULE
  python_bindings.cpp)
target_include_directories(${PROJECT_NAME}_python PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_python PUBLIC ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}_python
  PUBLIC ${PROJECT_NAME}
         Eigen3::Eigen
         Boost::serialization
         Boost::python
         Boost::numpy
         ${PCL_LIBRARIES}
         ${PYTHON_LIBRARIES}
         yaml-cpp
         boost_plugin_loader::boost_plugin_loader
         OpenMP::OpenMP_CXX)
set_target_properties(${PROJECT_NAME}_python
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages/${PROJECT_NAME})
list(APPEND TARGETS ${PROJECT_NAME} PARENT_SCOPE)
