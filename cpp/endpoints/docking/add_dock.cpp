#include "rclcpp/rclcpp.hpp"
#include "clearpath_dock_msgs/srv/add_dock.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
  std::string namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_dock_client");
  rclcpp::Client<clearpath_dock_msgs::srv::AddDock>::SharedPtr client =
    node->create_client<clearpath_dock_msgs::srv::AddDock>("/" + namespace_ + "/" + std::string("docking/dock_manager/add_dock"));

  // add your dock info (name, latitude, longitude, and orientation as a quaternion)
  auto request = std::make_shared<clearpath_dock_msgs::srv::AddDock::Request>();
  auto dock_info = clearpath_dock_msgs::msg::DockInfo();
  dock_info.name = "";
  dock_info.dock_template = "a300_side_dock";
  dock_info.latitude = 0.0;
  dock_info.longitude = 0.0;
  geometry_msgs::msg::Quaternion orientation;
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = 0.0;
  orientation.w = 1.0;
  dock_info.orientation = orientation;

  request->dock = dock_info;

  while (!client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("add_dock_client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("add_dock_client"), "/%s/docking/dock_manager/add_dock service not available, waiting again...", namespace_.c_str());
  }

  auto result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("add_dock_client"), "Added dock successfully!");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("add_dock_client"), "Failed to add dock!");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("add_dock_client"), "Failed to call service /%s/docking/dock_manager/add_dock", namespace_.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
