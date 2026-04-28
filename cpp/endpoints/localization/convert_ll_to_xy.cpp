#include "rclcpp/rclcpp.hpp"
#include "clearpath_localization_msgs/srv/convert_lat_lon_to_cartesian.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <yaml-cpp/yaml.h>

// TODO: Enter your latitude/longitude
const double LATITUDE = 0.0;
const double LONGITUDE = 0.0;

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
  std::string namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("convert_lat_lon_to_cartesian_client");
  rclcpp::Client<clearpath_localization_msgs::srv::ConvertLatLonToCartesian>::SharedPtr client =
    node->create_client<clearpath_localization_msgs::srv::ConvertLatLonToCartesian>("/" + namespace_ + "/" + std::string("localization/lat_lon_to_xy"));

  auto request = std::make_shared<clearpath_localization_msgs::srv::ConvertLatLonToCartesian::Request>();
  auto lat_lon_msg = sensor_msgs::msg::NavSatFix();
  lat_lon_msg.latitude = LATITUDE;
  lat_lon_msg.longitude = LONGITUDE;
  request->msg = lat_lon_msg;

  while (!client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("convert_lat_lon_to_cartesian_client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("convert_lat_lon_to_cartesian_client"), "/%s/localization/lat_lon_to_xy service not available, waiting again...", namespace_.c_str());
  }

  auto result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("convert_lat_lon_to_cartesian_client"), "Converted lat/lon to Cartesian successfully!");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("convert_lat_lon_to_cartesian_client"), "Failed to convert lat/lon to Cartesian");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("convert_lat_lon_to_cartesian_client"), "Failed to call service /%s/localization/lat_lon_to_xy", namespace_.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
