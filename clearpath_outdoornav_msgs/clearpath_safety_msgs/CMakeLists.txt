cmake_minimum_required(VERSION 3.8)
project(clearpath_safety_msgs)

## Find catkin macros and libraries

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/WatchdogConfigArray.msg
        msg/WatchdogConfigCommon.msg
        msg/WatchdogConfigCommunication.msg
        msg/WatchdogConfigHeartbeat.msg
        msg/WatchdogConfigInclination.msg
        msg/WatchdogConfigNodeletStatus.msg
        msg/WatchdogConfigNodeStatus.msg
        msg/WatchdogConfigNumPoints.msg
        msg/WatchdogConfigOdomCovariance.msg
        msg/WatchdogConfigRtkFix.msg
        msg/WatchdogConfigTopicData.msg
        msg/WatchdogConfigTrigger.msg
        msg/WatchdogStatus.msg
        msg/WatchdogStatusArray.msg
        srv/AddCommunicationWatchdog.srv
        srv/AddHeartbeatWatchdog.srv
        srv/AddInclinationWatchdog.srv
        srv/AddNodeletStatusWatchdog.srv
        srv/AddNodeStatusWatchdog.srv
        srv/AddNumPointsWatchdog.srv
        srv/AddOdomCovarianceWatchdog.srv
        srv/AddRtkFixWatchdog.srv
        srv/AddTopicDataWatchdog.srv
        srv/AddTriggerWatchdog.srv
        srv/RemoveWatchdog.srv
        srv/UpdateCommunicationWatchdog.srv
        srv/UpdateHeartbeatWatchdog.srv
        srv/UpdateInclinationWatchdog.srv
        srv/UpdateNodeletStatusWatchdog.srv
        srv/UpdateNodeStatusWatchdog.srv
        srv/UpdateNumPointsWatchdog.srv
        srv/UpdateOdomCovarianceWatchdog.srv
        srv/UpdateRtkFixWatchdog.srv
        srv/UpdateTopicDataWatchdog.srv
        srv/UpdateTriggerWatchdog.srv
    DEPENDENCIES
        std_msgs
)

ament_package()

install(FILES
  DESTINATION include/${PROJECT_NAME}
)
