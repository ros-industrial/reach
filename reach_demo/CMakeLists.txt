cmake_minimum_required(VERSION 3.5)
project(reach_demo)

find_package(ament_cmake REQUIRED)

#############
## Testing ##
#############
# no testing

#############
## Install ##
#############
install(DIRECTORY config launch model results
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
