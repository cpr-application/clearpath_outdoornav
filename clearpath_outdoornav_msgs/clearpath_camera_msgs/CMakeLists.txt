cmake_minimum_required(VERSION 3.8)
project(clearpath_camera_msgs)

## Find catkin macros and libraries
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        action/InspectPoi.action
        srv/Q62AreaZoom.srv
    DEPENDENCIES
        std_msgs
)

ament_package()

install(FILES
  DESTINATION include/${PROJECT_NAME}
)