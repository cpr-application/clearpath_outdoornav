cmake_minimum_required(VERSION 3.8)
project(clearpath_mission_manager_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(clearpath_navigation_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/NetworkEdgeReq.msg
        msg/NetworkMapState.msg
        msg/NetworkMapValidityState.msg
        msg/StorageState.msg
        srv/AddNetworkEdge.srv
        srv/AddNetworkPath.srv
        srv/AddNetworkPoint.srv
        srv/AddRemoveById.srv
        srv/AddRemoveTag.srv
        srv/CloneMission.srv
        srv/CloneNetworkMap.srv
        srv/CloneNetworkMission.srv
        srv/ClonePointOfInterest.srv
        srv/CreateMission.srv
        srv/CreateNetworkMap.srv
        srv/CreateNetworkMission.srv
        srv/CreatePointOfInterest.srv
        srv/CreateTask.srv
        srv/CreateWaypoint.srv
        srv/CreateWaypointFromPoi.srv
        srv/DeleteById.srv
        srv/DeleteEverything.srv
        srv/DeleteMultiple.srv
        srv/ExportData.srv
        srv/GetAllMissions.srv
        srv/GetAllNetworkMaps.srv
        srv/GetAllNetworkMissions.srv
        srv/GetAllPointsOfInterest.srv
        srv/GetAllTasks.srv
        srv/GetAllWaypoints.srv
        srv/GetEverything.srv
        srv/GetMission.srv
        srv/GetNetworkMap.srv
        srv/GetNetworkMission.srv
        srv/GetPointOfInterest.srv
        srv/GetTask.srv
        srv/GetWaypoint.srv
        srv/ImportData.srv
        srv/ImportMission.srv
        srv/ImportNetworkMap.srv
        srv/ImportNetworkMission.srv
        srv/SplitNetworkEdge.srv
        srv/UpdateMission.srv
        srv/UpdateNetworkEdge.srv
        srv/UpdateNetworkEdges.srv
        srv/UpdateNetworkMap.srv
        srv/UpdateNetworkMission.srv
        srv/UpdateNetworkPoint.srv
        srv/UpdateNetworkPoints.srv
        srv/UpdatePointOfInterest.srv
        srv/UpdateTask.srv
        srv/UpdateWaypoint.srv
        srv/ValidateNetworkMap.srv
    DEPENDENCIES
        clearpath_navigation_msgs
)

ament_package()

install(FILES
  DESTINATION include/${PROJECT_NAME}
)
