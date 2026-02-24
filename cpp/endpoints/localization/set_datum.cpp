#include "rclcpp/rclcpp.hpp"
#include "clearpath_localization_msgs/srv/set_datum.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <yaml-cpp/yaml.h>

// enter your datum latitude/longitude
const double DATUM_LAT = ;
const double DATUM_LON = ;

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
  std::string namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("set_datum_client");
  rclcpp::Client<clearpath_localization_msgs::srv::SetDatum>::SharedPtr client =
    node->create_client<clearpath_localization_msgs::srv::SetDatum>("/" + namespace_ + "/" + std::string("localization/set_datum"));

  auto request = std::make_shared<clearpath_localization_msgs::srv::SetDatum::Request>();
  request->lat = DATUM_LAT;
  request->lon = DATUM_LON;

  while (!client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("set_datum_client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("set_datum_client"), "/%s/localization/set_datum service not available, waiting again...", namespace_.c_str());
  }

  auto result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_datum_client"), "Datum set successfully!");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("set_datum_client"), "Failed to set datum!");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("set_datum_client"), "Failed to call service /%s/localization/set_datum", namespace_.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
