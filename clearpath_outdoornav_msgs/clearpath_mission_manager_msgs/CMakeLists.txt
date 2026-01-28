cmake_minimum_required(VERSION 3.8)
project(clearpath_mission_manager_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(clearpath_navigation_msgs REQUIRED)
find_package(clearpath_task_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/MapEdgeReq.msg
        msg/MapState.msg
        msg/MapValidityState.msg
        msg/StorageState.msg
        srv/AddMapEdge.srv
        srv/AddMapPath.srv
        srv/AddMapPoint.srv
        srv/AddRemoveById.srv
        srv/AddRemoveTag.srv
        srv/CloneMap.srv
        srv/CloneNetworkMission.srv
        srv/ClonePointOfInterest.srv
        srv/CreateMap.srv
        srv/CreateNetworkMission.srv
        srv/CreatePointOfInterest.srv
        srv/CreateTask.srv
        srv/CreateWaypoint.srv
        srv/CreateWaypointFromPoi.srv
        srv/DeleteById.srv
        srv/DeleteEverything.srv
        srv/DeleteMultiple.srv
        srv/ExportData.srv
        srv/GetAllMaps.srv
        srv/GetAllNetworkMissions.srv
        srv/GetAllPointsOfInterest.srv
        srv/GetAllTasks.srv
        srv/GetAllWaypoints.srv
        srv/GetEverything.srv
        srv/GetMap.srv
        srv/GetNetworkMission.srv
        srv/GetPointOfInterest.srv
        srv/GetTask.srv
        srv/GetWaypoint.srv
        srv/ImportData.srv
        srv/ImportMap.srv
        srv/ImportNetworkMission.srv
        srv/ImportTask.srv
        srv/SplitMapEdge.srv
        srv/UpdateMapEdge.srv
        srv/UpdateMapEdges.srv
        srv/UpdateMap.srv
        srv/UpdateNetworkMission.srv
        srv/UpdateMapPoint.srv
        srv/UpdateMapPoints.srv
        srv/UpdatePointOfInterest.srv
        srv/UpdateTask.srv
        srv/UpdateWaypoint.srv
        srv/ValidateMap.srv
    DEPENDENCIES
        clearpath_navigation_msgs
        clearpath_task_msgs
)

ament_package()

install(FILES
  DESTINATION include/${PROJECT_NAME}
)
