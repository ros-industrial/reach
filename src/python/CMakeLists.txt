# Add Python libraries
python_add_library(${PROJECT_NAME}_python MODULE python_bindings.cpp)
target_include_directories(${PROJECT_NAME}_python PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}_python
  PRIVATE ${PROJECT_NAME}
          Boost::python
          Boost::numpy
          ${PYTHON_LIBRARIES})
target_compile_definitions(${PROJECT_NAME}_python PRIVATE MODULE_NAME=${PROJECT_NAME})
set_target_properties(
  ${PROJECT_NAME}_python PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages
                                    OUTPUT_NAME ${PROJECT_NAME})
target_cxx_version(${PROJECT_NAME}_python PUBLIC VERSION 14)

# Add a hook for colcon to add the Python library directory to the PYTHONPATH environment variable
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv
     "prepend-non-duplicate;PYTHONPATH;lib/python3/dist-packages")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/python_path.dsv
        DESTINATION share/${PROJECT_NAME}/hook)

list(
  APPEND
  TARGETS
  ${PROJECT_NAME}_python
  PARENT_SCOPE)
