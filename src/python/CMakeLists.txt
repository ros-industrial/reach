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
set_target_properties(${PROJECT_NAME}_python PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_cxx_version(${PROJECT_NAME}_python PUBLIC VERSION 14)

execute_process(
  COMMAND "${Python_EXECUTABLE}" -c "if True:
    from distutils import sysconfig as sc
    print(sc.get_python_lib(prefix='', plat_specific=True))"
  OUTPUT_VARIABLE PYTHON_SITE
  OUTPUT_STRIP_TRAILING_WHITESPACE)

install(TARGETS ${PROJECT_NAME}_python DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE})
