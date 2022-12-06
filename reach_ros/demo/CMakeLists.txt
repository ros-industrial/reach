if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/demo.test)
endif()

foreach(dir config model results)
  install(DIRECTORY ${dir}/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/demo/${dir})
endforeach()
