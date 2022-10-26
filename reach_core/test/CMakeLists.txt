find_gtest()

add_executable(${PROJECT_NAME}_plugin_utest plugin_utest.cpp)
target_link_libraries(${PROJECT_NAME}_plugin_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
target_cxx_version(${PROJECT_NAME}_plugin_utest PUBLIC VERSION 14)
add_dependencies(run_tests ${PROJECT_NAME}_plugin_utest)
add_gtest_discover_tests(${PROJECT_NAME}_plugin_utest)

install(
  TARGETS ${PROJECT_NAME}_plugin_utest
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
