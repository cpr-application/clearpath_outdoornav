cmake_minimum_required(VERSION 3.20)
project(clearpath_diagnostics_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        action/CaptureDiagnostics.action
    DEPENDENCIES
        builtin_interfaces
)

ament_package()
