#include "rclcpp/rclcpp.hpp"
#include "clearpath_logger_msgs/srv/delete_log.hpp"

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

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("delete_log_client");
  rclcpp::Client<clearpath_logger_msgs::srv::DeleteLog>::SharedPtr client =
    node->create_client<clearpath_logger_msgs::srv::DeleteLog>("/" + namespace_ + "/" + std::string("log_manager/delete_log"));

  // enter the log uuid to delete
  auto request = std::make_shared<clearpath_logger_msgs::srv::DeleteLog::Request>();
  request->uuid = "";
  request->delete_media = false;
  request->purge_record = false;

while (!client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("delete_log_client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("delete_log_client"), "/%s/log_manager/delete_log service not available, waiting again...", namespace_.c_str());
  }

  auto result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("delete_log_client"), "Deleted log successfully!");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("delete_log_client"), "Failed to delete log!");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("delete_log_client"), "Failed to call service /%s/log_manager/delete_log", namespace_.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
