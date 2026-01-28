#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

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

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("resume_autonomy_client");
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client =
    node->create_client<std_srvs::srv::SetBool>("/" + namespace_ + "/" + std::string("autonomy/resume"));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  while (!client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("resume_autonomy_client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("resume_autonomy_client"), "/%s/autonomy/resume service not available, waiting again...", namespace_.c_str());
  }

  auto result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("resume_autonomy_client"), "Autonomy resumed!");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("resume_autonomy_client"), "Autonomy failed to resume!");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("resume_autonomy_client"), "Failed to call service /%s/autonomy/resume", namespace_.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
