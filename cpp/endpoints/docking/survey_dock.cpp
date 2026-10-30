#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "clearpath_dock_msgs/action/survey_dock.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

// TODO: Enter your dock info
const std::string DOCK_NAME = "";
const double TIMEOUT = 30.0;

using namespace std::chrono_literals;
using SurveyDock = clearpath_dock_msgs::action::SurveyDock;
using GoalHandleSurveyDock = rclcpp_action::ClientGoalHandle<SurveyDock>;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
  std::string namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();
  const std::string action_name = "/" + namespace_ + "/docking/dock_localizer/survey_dock";

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("survey_dock_client");
  auto logger = node->get_logger();
  auto client = rclcpp_action::create_client<SurveyDock>(node, action_name);

  while (!client->wait_for_action_server(2s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger, "Interrupted while waiting for the action server. Exiting.");
      return 0;
    }
    RCLCPP_INFO(logger, "%s action server not available, waiting again...", action_name.c_str());
  }

  auto goal_msg = SurveyDock::Goal();
  goal_msg.dock_name = DOCK_NAME;
  goal_msg.timeout = TIMEOUT;

  auto send_goal_options = rclcpp_action::Client<SurveyDock>::SendGoalOptions();
  send_goal_options.feedback_callback = [node, logger](
    GoalHandleSurveyDock::SharedPtr,
    const std::shared_ptr<const SurveyDock::Feedback> feedback)
  {
    RCLCPP_INFO_THROTTLE(logger, *node->get_clock(), 1000, "%s: %u/%u samples",
      feedback->message.c_str(), feedback->samples_collected, feedback->samples_required);
  };

  auto goal_handle_future = client->async_send_goal(goal_msg, send_goal_options);
  if (rclcpp::spin_until_future_complete(node, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Failed to send goal to %s", action_name.c_str());
    rclcpp::shutdown();
    return 1;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(logger, "Survey dock goal was rejected (is a survey already in progress?)");
    rclcpp::shutdown();
    return 1;
  }

  auto result_future = client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Failed to get survey dock result");
    rclcpp::shutdown();
    return 1;
  }

  auto wrapped_result = result_future.get();
  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger, "Surveyed dock successfully!");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(logger, "Survey dock cancelled");
      break;
    default:
      RCLCPP_ERROR(logger, "Failed to survey dock: %s", wrapped_result.result->message.c_str());
      break;
  }

  rclcpp::shutdown();
  return 0;
}
