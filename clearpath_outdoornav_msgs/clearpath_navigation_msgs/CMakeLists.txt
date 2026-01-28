cmake_minimum_required(VERSION 3.8)
project(clearpath_navigation_msgs)

## Find catkin macros and libraries

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        action/ExecuteMissionByUuid.action
        action/ExecuteNetworkGoTo.action
        action/ExecuteNetworkMissionByUuid.action
        action/ExecuteNetworkMissionFromGoal.action
        action/GetWaypointPath.action
        action/Mission.action
        action/RecordPath.action
        action/SmoothPath.action
        action/UITask.action
        msg/AutonomyStatus.msg
        msg/DistanceToGoal.msg
        msg/Metrics.msg
        msg/Mission.msg
        msg/MissionStatus.msg
        msg/MotionState.msg
        msg/NavigationState.msg
        msg/NetworkEdge.msg
        msg/NetworkGoalState.msg
        msg/NetworkMap.msg
        msg/NetworkMission.msg
        msg/NetworkMissionSegment.msg
        msg/NetworkPoint.msg
        msg/OdomIntent.msg
        msg/PointOfInterest.msg
        msg/PointVector.msg
        msg/Progress.msg
        msg/ScanPoints.msg
        msg/MissionPreview.msg
        msg/GoToPreview.msg
        msg/Task.msg
        msg/TrackError.msg
        msg/Vector2D.msg
        msg/Vector2D64.msg
        msg/Waypoint.msg
        msg/WaypointStatus.msg
        srv/AutonomyTaskSrv.srv
        srv/SetDelayCompensation.srv
        srv/SetPathShifter.srv
        srv/SetPathSmoother.srv
        srv/SetStopDistance.srv
        srv/StartGoToPreviewer.srv
        srv/StartMissionPreviewer.srv
    DEPENDENCIES
        builtin_interfaces
        geometry_msgs
        nav_msgs
        std_msgs
)

ament_package()

install(FILES
  DESTINATION include/${PROJECT_NAME}
)
