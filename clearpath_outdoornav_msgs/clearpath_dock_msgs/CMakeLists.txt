cmake_minimum_required(VERSION 3.8)
project(clearpath_dock_msgs)

## Find catkin macros and libraries

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(clearpath_navigation_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        action/MapDock.action
        action/Dock.action
        action/Undock.action
        msg/DockInfoArray.msg
        msg/DockInfo.msg
        msg/DockTemplate.msg
#        msg/DockProperties.msg
        msg/DockState.msg
        msg/TargetTrackerState.msg
        msg/DockDatabase.msg
        srv/RunDocking.srv
        srv/RunUndocking.srv
        srv/AddDock.srv
        srv/AddDockCurrentPose.srv
        srv/ExportData.srv
        srv/GetDockDatabase.srv
        srv/GetDock.srv
        srv/ImportData.srv
        srv/RemoveDock.srv
        srv/UpdateDock.srv
        srv/GetDockPoses.srv
        srv/SurveyDock.srv
    DEPENDENCIES
        geometry_msgs
        clearpath_navigation_msgs
)

ament_package()

install(FILES
  DESTINATION include/${PROJECT_NAME}
)
