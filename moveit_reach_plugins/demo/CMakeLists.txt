if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/demo.test)
endif()

install(
  DIRECTORY config
            model
            results
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
