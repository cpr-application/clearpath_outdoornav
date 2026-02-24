#include "rclcpp/rclcpp.hpp"
#include "clearpath_task_msgs/srv/execute_task.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

// TODO: Enter your task id
const std::string TASK_ID = "";

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
  std::string namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("execute_task_srv_client");
  rclcpp::Client<clearpath_task_msgs::srv::ExecuteTask>::SharedPtr client =
    node->create_client<clearpath_task_msgs::srv::ExecuteTask>("/" + namespace_ + "/" + std::string("execute_task"));

  auto request = std::make_shared<clearpath_task_msgs::srv::ExecuteTask::Request>();
  request->task_id = TASK_ID;

  while (!client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("execute_task_srv_client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("execute_task_srv_client"), "/%s/execute_task service not available, waiting again...", namespace_.c_str());
  }

  auto result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("execute_task_srv_client"), "Task executed successfully!");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("execute_task_srv_client"), "Failed to execute task!");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("execute_task_srv_client"), "Failed to call service /%s/execute_task", namespace_.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
