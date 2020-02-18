cmake_minimum_required(VERSION 2.8.3)
project(reach_demo)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch model results
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
