cmake_minimum_required(VERSION 3.8)
project(clearpath_localization_msgs)

## Find catkin macros and libraries

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        action/SurveyBaseStation.action
        msg/GNSSStatus.msg
        msg/LocalizationStatus.msg
        msg/SurveyProperties.msg
        msg/XvnStatus.msg
        srv/ConvertCartesianToLatLon.srv
        srv/ConvertCartesianToLatLonArray.srv
        srv/ConvertLatLonToCartesian.srv
        srv/ConvertLatLonToCartesianArray.srv
        srv/ResetLocalization.srv
        srv/SetDatum.srv
    DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

ament_package()

install(FILES
  DESTINATION include/${PROJECT_NAME}
)
