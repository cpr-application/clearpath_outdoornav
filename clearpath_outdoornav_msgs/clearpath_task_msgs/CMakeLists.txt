cmake_minimum_required(VERSION 3.20)
project(clearpath_task_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        action/ExecuteTask.action
        action/CustomTask.action
        msg/TaskStatus.msg
        msg/Task.msg
        srv/ExecuteTask.srv
        srv/CustomTask.srv
    DEPENDENCIES
        builtin_interfaces
        std_msgs
)

ament_package()

install(FILES
  DESTINATION include/${PROJECT_NAME}
)
