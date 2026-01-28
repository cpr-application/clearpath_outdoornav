cmake_minimum_required(VERSION 3.8)
project(clearpath_configuration_msgs)

## Find catkin macros and libraries

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/AutonomyConfig.msg
        msg/PlatformConfig.msg
)

ament_package()

install(FILES
  DESTINATION include/${PROJECT_NAME}
)
