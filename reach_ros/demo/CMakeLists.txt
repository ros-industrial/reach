#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest(test/demo.test)
#endif()

foreach(dir config model results)
  install(DIRECTORY ${dir}/ DESTINATION share/${PROJECT_NAME}/demo/${dir})
endforeach()
