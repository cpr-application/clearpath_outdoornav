cmake_minimum_required(VERSION 3.20)
project(clearpath_navigation_msgs)

## Find catkin macros and libraries

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(clearpath_task_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        action/ExecuteGoTo.action
        action/ExecuteGoToPOI.action
        action/ExecuteMission.action
        action/ExecuteMissionFromGoal.action
        action/InspectPoi.action
        action/RecordPath.action
        msg/AutonomyConfig.msg
        msg/AutonomyStatus.msg
        msg/MissionStatus.msg
        msg/MapEdge.msg
        msg/MapGoalState.msg
        msg/Map.msg
        msg/Mission.msg
        msg/MissionSegment.msg
        msg/MapPoint.msg
        msg/NetworkMission.msg
        msg/PointOfInterest.msg
        msg/MissionPreview.msg
        msg/GoToPreview.msg
        msg/Vector2D64.msg
        msg/Waypoint.msg
        msg/WaypointStatus.msg
        srv/Q62AreaZoom.srv
        srv/StartGoToPreviewer.srv
        srv/StartMissionPreviewer.srv
    DEPENDENCIES
        builtin_interfaces
        geometry_msgs
        nav_msgs
        std_msgs
        clearpath_task_msgs
)

ament_package()

install(FILES
  DESTINATION include/${PROJECT_NAME}
)
