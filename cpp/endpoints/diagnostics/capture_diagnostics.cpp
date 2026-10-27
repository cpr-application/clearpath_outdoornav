#include <atomic>
#include <csignal>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include "clearpath_diagnostics_msgs/action/capture_diagnostics.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// TODO: Optionally name the capture (letters, digits, '.', '_' and '-').
// Empty uses nav_diag_<namespace>_<timestamp>.
const std::string CAPTURE_NAME = "";
// Monitoring window in seconds; 0 uses the server's default (60 s)
const double DURATION = 0.0;
const bool INCLUDE_HEAVY_TOPICS = false;
const bool RECORD_BAG = false;

namespace
{
std::atomic<bool> g_cancel_requested{false};

void handle_sigint(int)
{
  g_cancel_requested = true;
}
}  // namespace

namespace capture_diagnostics_client
{
class CaptureDiagnosticsActionClient : public rclcpp::Node
{
public:
  using CaptureDiagnostics = clearpath_diagnostics_msgs::action::CaptureDiagnostics;
  using GoalHandleCaptureDiagnostics = rclcpp_action::ClientGoalHandle<CaptureDiagnostics>;

  explicit CaptureDiagnosticsActionClient(const std::string & robot_namespace)
  : Node("capture_diagnostics"), namespace_(robot_namespace)
  {
    action_name_ = "/" + namespace_ + "/outdoornav/diagnostics/capture";
    client_ptr_ = rclcpp_action::create_client<CaptureDiagnostics>(this, action_name_);
  }

  bool send_goal()
  {
    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "%s action server not available", action_name_.c_str());
      return false;
    }

    auto goal_msg = CaptureDiagnostics::Goal();
    goal_msg.name = CAPTURE_NAME;
    goal_msg.duration = DURATION;
    goal_msg.include_heavy_topics = INCLUDE_HEAVY_TOPICS;
    goal_msg.record_bag = RECORD_BAG;

    auto send_goal_options = rclcpp_action::Client<CaptureDiagnostics>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](const GoalHandleCaptureDiagnostics::SharedPtr & goal_handle)
      {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Capture Goal rejected (is another capture running?)");
          done_ = true;
        } else {
          goal_handle_ = goal_handle;
          RCLCPP_INFO(
            get_logger(),
            "Capture Goal accepted. Press Ctrl-C to end the monitoring window early.");
        }
      };

    send_goal_options.feedback_callback = [this](
      GoalHandleCaptureDiagnostics::SharedPtr,
      const std::shared_ptr<const CaptureDiagnostics::Feedback> feedback)
      {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000, "%s (%.0f%%, %.0fs): %s",
          phase_name(feedback->phase), feedback->progress * 100.0, feedback->time_elapsed,
          feedback->message.c_str());
      };

    send_goal_options.result_callback =
      [this](const GoalHandleCaptureDiagnostics::WrappedResult & result)
      {
        const auto & capture = result.result;
        const bool written = capture && capture->success;
        if ((result.code == rclcpp_action::ResultCode::SUCCEEDED ||
          result.code == rclcpp_action::ResultCode::CANCELED) && written)
        {
          RCLCPP_INFO(get_logger(), "[Capture Goal Result] %s", capture->message.c_str());
          if (capture->has_error_findings) {
            RCLCPP_WARN(
              get_logger(), "[Capture Goal Result] The capture summary reports ERROR findings");
          }
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          RCLCPP_INFO(
            get_logger(), "[Capture Goal Result] Cancelled: %s",
            capture ? capture->message.c_str() : "");
        } else {
          RCLCPP_ERROR(
            get_logger(), "[Capture Goal Result] Failed: %s",
            capture ? capture->message.c_str() : "no result");
        }
        done_ = true;
      };

    client_ptr_->async_send_goal(goal_msg, send_goal_options);
    return true;
  }

  void cancel()
  {
    if (!goal_handle_) {
      done_ = true;
      return;
    }
    RCLCPP_INFO(get_logger(), "Ending the capture early; waiting for results...");
    client_ptr_->async_cancel_goal(goal_handle_);
  }

  bool done() const {return done_;}

private:
  static const char * phase_name(uint8_t phase)
  {
    switch (phase) {
      case CaptureDiagnostics::Feedback::PHASE_DISCOVERING: return "Discovering";
      case CaptureDiagnostics::Feedback::PHASE_MONITORING: return "Monitoring";
      case CaptureDiagnostics::Feedback::PHASE_COLLECTING: return "Collecting";
      case CaptureDiagnostics::Feedback::PHASE_FINALIZING: return "Finalizing";
      default: return "Unknown phase";
    }
  }

  rclcpp_action::Client<CaptureDiagnostics>::SharedPtr client_ptr_;
  GoalHandleCaptureDiagnostics::SharedPtr goal_handle_;
  std::string namespace_;
  std::string action_name_;
  std::atomic<bool> done_{false};
};  // class CaptureDiagnosticsActionClient

}  // namespace capture_diagnostics_client

int main(int argc, char ** argv)
{
  // Handle Ctrl-C ourselves so the capture can be cancelled and its results still received
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, handle_sigint);

  YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
  const auto robot_namespace = robot_config["system"]["ros2"]["namespace"].as<std::string>();

  auto client = std::make_shared<capture_diagnostics_client::CaptureDiagnosticsActionClient>(
    robot_namespace);
  if (client->send_goal()) {
    bool cancel_sent = false;
    while (rclcpp::ok() && !client->done()) {
      if (g_cancel_requested) {
        if (cancel_sent) {
          break;
        }
        cancel_sent = true;
        g_cancel_requested = false;
        client->cancel();
      }
      rclcpp::spin_some(client);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  rclcpp::shutdown();
  return 0;
}
