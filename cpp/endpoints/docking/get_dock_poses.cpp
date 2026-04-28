#include "rclcpp/rclcpp.hpp"
#include "clearpath_dock_msgs/srv/get_dock_poses.hpp"

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

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_dock_poses_client");
  rclcpp::Client<clearpath_dock_msgs::srv::GetDockPoses>::SharedPtr client =
    node->create_client<clearpath_dock_msgs::srv::GetDockPoses>("/" + namespace_ + "/" + std::string("docking/dock_localizer/get_dock_poses"));

  auto request = std::make_shared<clearpath_dock_msgs::srv::GetDockPoses::Request>();

  while (!client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("get_dock_poses_client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("get_dock_poses_client"), "/%s/docking/dock_localizer/get_dock_poses service not available, waiting again...", namespace_.c_str());
  }

  auto result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("get_dock_poses_client"), "Docks poses retrieved successfully!");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("get_dock_poses_client"), "Failed to retrieve docks poses: %s", result.get()->message.c_str());
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("get_dock_poses_client"), "Failed to call service /%s/docking/dock_localizer/get_dock_poses", namespace_.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
