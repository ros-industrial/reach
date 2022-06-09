cmake_minimum_required(VERSION 3.5)
project(reach_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-W -Wall -Wextra
      -Wwrite-strings -Wunreachable-code -Wpointer-arith
      -Winit-self -Wredundant-decls
      -Wno-unused-parameter -Wno-unused-function)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    builtin_interfaces
    std_msgs
    geometry_msgs
  rosidl_default_generators
  rosidl_default_runtime
    sensor_msgs
    moveit_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(msg_files
    "msg/ReachRecord.msg"
    "msg/ReachDatabase.msg"
)

set(srv_files
    "srv/LoadPointCloud.srv"
)

#generate_messages(
#  DEPENDENCIES
#    geometry_msgs
#    sensor_msgs
#)

#catkin_package(
#  CATKIN_DEPENDS
#    message_runtime
#    geometry_msgs
#    sensor_msgs
#)

###########
## Build ##
###########


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    sensor_msgs
    moveit_msgs
)


#############
## Install ##
#############

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()