#include "rclcpp/rclcpp.hpp"
#include "clearpath_dock_msgs/srv/update_dock.hpp"

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

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("update_dock_client");
  rclcpp::Client<clearpath_dock_msgs::srv::UpdateDock>::SharedPtr client =
    node->create_client<clearpath_dock_msgs::srv::UpdateDock>("/" + namespace_ + "/" + std::string("docking/dock_manager/update"));

  auto request = std::make_shared<clearpath_dock_msgs::srv::UpdateDock::Request>();

  while (!client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("update_dock_client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("update_dock_client"), "/%s/docking/dock_manager/update service not available, waiting again...", namespace_.c_str());
  }

  auto result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("update_dock_client"), "Dock updated successfully!");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("update_dock_client"), "Failed to update dock!");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("update_dock_client"), "Failed to call service /%s/docking/dock_manager/update", namespace_.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
