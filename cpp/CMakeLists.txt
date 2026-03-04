cmake_minimum_required(VERSION 3.20)
project(clearpath_outdoornav_cpp_api)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(clearpath_dock_msgs REQUIRED)
find_package(clearpath_localization_msgs REQUIRED)
find_package(clearpath_navigation_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

set(dependencies
  clearpath_dock_msgs
  clearpath_localization_msgs
  clearpath_navigation_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  std_srvs
  yaml_cpp_vendor
)

include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_LIBRARIES}
)

# Execute Task Action
add_executable(execute_task_action
  endpoints/autonomy/execute_task_action.cpp
)
target_include_directories(execute_task_action PUBLIC include)
ament_target_dependencies(execute_task_action ${dependencies})
target_link_libraries(execute_task_action
  yaml-cpp::yaml-cpp
)

# Execute Task Service
add_executable(execute_task_srv
  endpoints/autonomy/execute_task_srv.cpp
)
target_include_directories(execute_task_srv PUBLIC include)
ament_target_dependencies(execute_task_srv ${dependencies})
target_link_libraries(execute_task_srv
  yaml-cpp::yaml-cpp
)

# GoTo POI
add_executable(goto_poi
  endpoints/autonomy/goto_poi.cpp
)
target_include_directories(goto_poi PUBLIC include)
ament_target_dependencies(goto_poi ${dependencies})
target_link_libraries(goto_poi
  yaml-cpp::yaml-cpp
)

# GoTo
add_executable(goto
  endpoints/autonomy/goto.cpp
)
target_include_directories(goto PUBLIC include)
ament_target_dependencies(goto ${dependencies})
target_link_libraries(goto
  yaml-cpp::yaml-cpp
)

# Mission From Goal
add_executable(mission_from_goal
  endpoints/autonomy/mission_from_goal.cpp
)
target_include_directories(mission_from_goal PUBLIC include)
ament_target_dependencies(mission_from_goal ${dependencies})
target_link_libraries(mission_from_goal
  yaml-cpp::yaml-cpp
)

# Mission
add_executable(mission
  endpoints/autonomy/mission.cpp
)
target_include_directories(mission PUBLIC include)
ament_target_dependencies(mission ${dependencies})
target_link_libraries(mission
  yaml-cpp::yaml-cpp
)

# Pause Autonomy
add_executable(pause_autonomy
  endpoints/autonomy/pause_autonomy.cpp
)
target_include_directories(pause_autonomy PUBLIC include)
ament_target_dependencies(pause_autonomy ${dependencies})
target_link_libraries(pause_autonomy
  yaml-cpp::yaml-cpp
)

# Resume Autonomy
add_executable(resume_autonomy
  endpoints/autonomy/resume_autonomy.cpp
)
target_include_directories(resume_autonomy PUBLIC include)
ament_target_dependencies(resume_autonomy ${dependencies})
target_link_libraries(resume_autonomy
  yaml-cpp::yaml-cpp
)

# Stop Autonomy
add_executable(stop_autonomy
  endpoints/autonomy/stop_autonomy.cpp
)
target_include_directories(stop_autonomy PUBLIC include)
ament_target_dependencies(stop_autonomy ${dependencies})
target_link_libraries(stop_autonomy
  yaml-cpp::yaml-cpp
)

# Add Dock
add_executable(add_dock
  endpoints/docking/add_dock.cpp
)
target_include_directories(add_dock PUBLIC include)
ament_target_dependencies(add_dock ${dependencies})
target_link_libraries(add_dock
  yaml-cpp::yaml-cpp
)

# Local Dock
add_executable(local_dock
  endpoints/docking/local_dock.cpp
)
target_include_directories(local_dock PUBLIC include)
ament_target_dependencies(local_dock ${dependencies})
target_link_libraries(local_dock
  yaml-cpp::yaml-cpp
)

# Map Dock
add_executable(map_dock
  endpoints/docking/map_dock.cpp
)
target_include_directories(map_dock PUBLIC include)
ament_target_dependencies(map_dock ${dependencies})
target_link_libraries(map_dock
  yaml-cpp::yaml-cpp
)

# Survey Dock
add_executable(survey_dock
  endpoints/docking/survey_dock.cpp
)
target_include_directories(survey_dock PUBLIC include)
ament_target_dependencies(survey_dock ${dependencies})
target_link_libraries(survey_dock
  yaml-cpp::yaml-cpp
)

# Undock
add_executable(undock
  endpoints/docking/undock.cpp
)
target_include_directories(undock PUBLIC include)
ament_target_dependencies(undock ${dependencies})
target_link_libraries(undock
  yaml-cpp::yaml-cpp
)

# Monitor Localization
add_executable(monitor_location
  endpoints/localization/monitor_location.cpp
)
target_include_directories(monitor_location PUBLIC include)
ament_target_dependencies(monitor_location ${dependencies})
target_link_libraries(monitor_location
  yaml-cpp::yaml-cpp
)

# Set Datum
add_executable(set_datum
  endpoints/localization/set_datum.cpp
)
target_include_directories(set_datum PUBLIC include)
ament_target_dependencies(set_datum ${dependencies})
target_link_libraries(set_datum
  yaml-cpp::yaml-cpp
)

# Delete Log
add_executable(delete_log
  endpoints/logger/delete_log.cpp
)
target_include_directories(delete_log PUBLIC include)
ament_target_dependencies(delete_log ${dependencies})
target_link_libraries(delete_log
  yaml-cpp::yaml-cpp
)

# Start Recording
add_executable(start_recording
  endpoints/logger/start_recording.cpp
)
target_include_directories(start_recording PUBLIC include)
ament_target_dependencies(start_recording ${dependencies})
target_link_libraries(start_recording
  yaml-cpp::yaml-cpp
)

# Stop Recording
add_executable(stop_recording
  endpoints/logger/stop_recording.cpp
)
target_include_directories(stop_recording PUBLIC include)
ament_target_dependencies(stop_recording ${dependencies})
target_link_libraries(stop_recording
  yaml-cpp::yaml-cpp
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS 
  execute_task_action
  execute_task_srv
  goto_poi
  goto
  mission_from_goal
  mission
  pause_autonomy
  resume_autonomy
  stop_autonomy
  add_dock
  local_dock
  map_dock
  survey_dock
  undock
  monitor_location
  set_datum
  delete_log
  start_recording
  stop_recording
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_dependencies(
  ${dependencies}
  yaml-cpp
)

#rclcpp_components_register_node(cpp_client PLUGIN "goto_poi_action_client::ExecuteGoToPOIActionClient" EXECUTABLE api_endpoint)

ament_package()