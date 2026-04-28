#include "rclcpp/rclcpp.hpp"
#include "clearpath_control_msgs/srv/set_control_mode.hpp"
#include "clearpath_control_msgs/msg/control_mode.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <yaml-cpp/yaml.h>

// enter your control mode
const int CONTROL_MODE = clearpath_control_msgs::msg::ControlMode::MANUAL; // options: MANUAL, AUTONOMOUS, NEUTRAL. See clearpath_control_msgs/msg/ControlMode.msg for more details.

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
  std::string namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("set_control_mode_client");
  rclcpp::Client<clearpath_control_msgs::srv::SetControlMode>::SharedPtr client =
    node->create_client<clearpath_control_msgs::srv::SetControlMode>("/" + namespace_ + "/" + std::string("control_selection/set_mode"));

  auto request = std::make_shared<clearpath_control_msgs::srv::SetControlMode::Request>();
  request->mode.mode = CONTROL_MODE;

  while (!client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("set_control_mode_client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("set_control_mode_client"), "/%s/control_selection/set_mode service not available, waiting again...", namespace_.c_str());
  }

  auto result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_control_mode_client"), "Control mode set successfully!");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("set_control_mode_client"), "Failed to set control mode!");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("set_control_mode_client"), "Failed to call service /%s/control_selection/set_mode", namespace_.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
