# Add Python libraries
Python_add_library(
  ${PROJECT_NAME}_python
  MODULE
  python_bindings.cpp)
target_include_directories(${PROJECT_NAME}_python PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}_python
  PRIVATE ${PROJECT_NAME}
          Boost::python
          Boost::numpy
          ${PYTHON_LIBRARIES})
set_target_properties(${PROJECT_NAME}_python
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages)

list(APPEND TARGETS ${PROJECT_NAME}_python PARENT_SCOPE)
