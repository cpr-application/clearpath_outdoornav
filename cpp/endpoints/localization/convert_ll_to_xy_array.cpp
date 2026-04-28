#include "rclcpp/rclcpp.hpp"
#include "clearpath_localization_msgs/srv/convert_lat_lon_to_cartesian_array.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <yaml-cpp/yaml.h>

// TODO: Enter your latitude/longitude
const double LATITUDE1 = 0.0;
const double LONGITUDE1 = 0.0;
const double LATITUDE2 = 0.0;
const double LONGITUDE2 = 0.0;

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
  std::string namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("convert_lat_lon_to_cartesian_array_client");
  rclcpp::Client<clearpath_localization_msgs::srv::ConvertLatLonToCartesianArray>::SharedPtr client =
    node->create_client<clearpath_localization_msgs::srv::ConvertLatLonToCartesianArray>("/" + namespace_ + "/" + std::string("localization/lat_lon_to_xy_array"));

  auto request = std::make_shared<clearpath_localization_msgs::srv::ConvertLatLonToCartesianArray::Request>();
  auto lat_lon_msg1 = sensor_msgs::msg::NavSatFix();
  lat_lon_msg1.latitude = LATITUDE1;
  lat_lon_msg1.longitude = LONGITUDE1;
  auto lat_lon_msg2 = sensor_msgs::msg::NavSatFix();
  lat_lon_msg2.latitude = LATITUDE2;
  lat_lon_msg2.longitude = LONGITUDE2;
  request->msg = {lat_lon_msg1, lat_lon_msg2};

  while (!client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("convert_lat_lon_to_cartesian_array_client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("convert_lat_lon_to_cartesian_array_client"), "/%s/localization/lat_lon_to_xy_array service not available, waiting again...", namespace_.c_str());
  }

  auto result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("convert_lat_lon_to_cartesian_array_client"), "Converted lat/lon array to Cartesian successfully!");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("convert_lat_lon_to_cartesian_array_client"), "Failed to convert lat/lon array to Cartesian");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("convert_lat_lon_to_cartesian_array_client"), "Failed to call service /%s/localization/lat_lon_to_xy_array", namespace_.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
