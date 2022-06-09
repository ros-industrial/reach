cmake_minimum_required(VERSION 2.8.3)
project(reach_demo)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED)

catkin_package()

# ######################################################################################################################
# Testing ##
# ######################################################################################################################
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/demo.test)
endif()

# ######################################################################################################################
# Install ##
# ######################################################################################################################
install(
  DIRECTORY config
            launch
            model
            results
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
