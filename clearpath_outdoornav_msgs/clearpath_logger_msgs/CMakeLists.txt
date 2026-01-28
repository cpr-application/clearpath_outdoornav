cmake_minimum_required(VERSION 3.8)
project(clearpath_logger_msgs)

## Find catkin macros and libraries
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/Event.msg
        msg/EventLog.msg
        msg/EventLogs.msg
        msg/EventTime.msg
        srv/DeleteLog.srv
        srv/GetAllLogs.srv
        srv/DownloadLog.srv
        srv/RecordEvent.srv
        srv/RecordCustomEvent.srv
        srv/RecordErrorEvent.srv
        srv/RecordLocationEvent.srv
        srv/RecordMediaEvent.srv
        srv/RecordPowerEvent.srv
        srv/StartRecording.srv
        srv/StopRecording.srv
    DEPENDENCIES
        geometry_msgs
        nav_msgs
        sensor_msgs
        std_msgs
)

ament_package()

install(FILES
  DESTINATION include/${PROJECT_NAME}
)
