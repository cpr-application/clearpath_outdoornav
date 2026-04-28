#include "rclcpp/rclcpp.hpp"
#include "clearpath_localization_msgs/srv/convert_cartesian_to_lat_lon_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <yaml-cpp/yaml.h>

// TODO: Enter your Cartesian coordinates
const double X1 = 0.0;
const double Y1 = 0.0;
const double Z1 = 0.0;

const double X2 = 0.0;
const double Y2 = 0.0;
const double Z2 = 0.0;

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
  std::string namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("convert_xy_to_lat_lon_array_client");
  rclcpp::Client<clearpath_localization_msgs::srv::ConvertCartesianToLatLonArray>::SharedPtr client =
    node->create_client<clearpath_localization_msgs::srv::ConvertCartesianToLatLonArray>("/" + namespace_ + "/" + std::string("localization/xy_to_lat_lon_array"));

  auto request = std::make_shared<clearpath_localization_msgs::srv::ConvertCartesianToLatLonArray::Request>();
  auto cartesian_msg1 = geometry_msgs::msg::PoseStamped();
  cartesian_msg1.pose.position.x = X1;
  cartesian_msg1.pose.position.y = Y1;
  cartesian_msg1.pose.position.z = Z1;

  auto cartesian_msg2 = geometry_msgs::msg::PoseStamped();
  cartesian_msg2.pose.position.x = X2;
  cartesian_msg2.pose.position.y = Y2;
  cartesian_msg2.pose.position.z = Z2;

  request->msg.push_back(cartesian_msg1);
  request->msg.push_back(cartesian_msg2);

  while (!client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("convert_xy_to_lat_lon_array_client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("convert_xy_to_lat_lon_array_client"), "/%s/localization/xy_to_lat_lon_array service not available, waiting again...", namespace_.c_str());
  }

  auto result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("convert_xy_to_lat_lon_array_client"), "Converted xy to lat/lon successfully!");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("convert_xy_to_lat_lon_array_client"), "Failed to convert xy to lat/lon");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("convert_xy_to_lat_lon_array_client"), "Failed to call service /%s/localization/xy_to_lat_lon_array", namespace_.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
