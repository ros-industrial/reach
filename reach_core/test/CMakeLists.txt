find_gtest()

add_executable(${PROJECT_NAME}_utest reach_study_utest.cpp)
target_link_libraries(${PROJECT_NAME}_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_definitions(${PROJECT_NAME}_utest
                           PRIVATE TEST_CONFIG_FILE="${CMAKE_CURRENT_SOURCE_DIR}/reach_study.yaml")
target_cxx_version(${PROJECT_NAME}_utest PUBLIC VERSION 14)
add_dependencies(run_tests ${PROJECT_NAME}_utest)
add_gtest_discover_tests(${PROJECT_NAME}_utest)

install(
  TARGETS ${PROJECT_NAME}_utest
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(FILES part.pcd reach_study.yaml DESTINATION share/${PROJECT_NAME}/test)
