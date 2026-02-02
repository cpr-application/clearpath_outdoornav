cmake_minimum_required(VERSION 3.20)
project(clearpath_control_msgs)

## Find catkin macros and libraries

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/ControlMode.msg
        msg/ControlSelectionState.msg
        msg/ControlState.msg
        srv/SetControlMode.srv
)

ament_package()

install(FILES
  DESTINATION include/${PROJECT_NAME}
)
