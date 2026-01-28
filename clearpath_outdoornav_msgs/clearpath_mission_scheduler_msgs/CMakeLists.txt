cmake_minimum_required(VERSION 3.8)
project(clearpath_mission_scheduler_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(clearpath_navigation_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        action/RunScheduleByUuid.action
        msg/NextSchedule.msg
        msg/Schedule.msg
        msg/StorageState.msg
        msg/UtcTime.msg
        srv/CloneSchedule.srv
        srv/CreateSchedule.srv
        srv/EnableSchedule.srv
        srv/ExportData.srv
        srv/GetAllSchedules.srv
        srv/GetNextSchedule.srv
        srv/GetSchedule.srv
        srv/ImportData.srv
        srv/UpdateSchedule.srv
    DEPENDENCIES
        builtin_interfaces
        clearpath_navigation_msgs
)

ament_package()

install(FILES
  DESTINATION include/${PROJECT_NAME}
)
