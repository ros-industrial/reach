cmake_minimum_required(VERSION 3.5)
project(reach_demo)

find_package(ament_cmake REQUIRED)

#############
## Testing ##
#############
#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest(test/demo.test)
#endif()

#############
## Install ##
#############
install(DIRECTORY config launch model results
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
