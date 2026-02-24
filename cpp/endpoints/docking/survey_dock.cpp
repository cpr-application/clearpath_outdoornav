#include "rclcpp/rclcpp.hpp"
#include "clearpath_dock_msgs/srv/survey_dock.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <yaml-cpp/yaml.h>

const std::string DOCK_NAME = "";
const double TIMEOUT = 30.0;

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
  std::string namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("survey_dock_client");
  rclcpp::Client<clearpath_dock_msgs::srv::SurveyDock>::SharedPtr client =
    node->create_client<clearpath_dock_msgs::srv::SurveyDock>("/" + namespace_ + "/" + std::string("docking/dock_localizer/survey_dock"));

  auto request = std::make_shared<clearpath_dock_msgs::srv::SurveyDock::Request>();
  request->dock_name = DOCK_NAME;
  request->timeout = TIMEOUT;

  while (!client->wait_for_service(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("survey_dock_client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("survey_dock_client"), "/%s/docking/dock_localizer/survey_dock service not available, waiting again...", namespace_.c_str());
  }

  auto result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("survey_dock_client"), "Surveyed dock successfully!");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("survey_dock_client"), "Failed to survey dock!");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("survey_dock_client"), "Failed to call service /%s/docking/dock_localizer/survey_dock", namespace_.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
