#include <functional>
#include <future>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include "clearpath_dock_msgs/action/dock.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// TODO: Enter your dock name
const std::string DOCK_NAME = "lma02_doghouse_dock";

namespace dock_action_client
{
class DockActionClient : public rclcpp::Node
{
public:
  using Dock = clearpath_dock_msgs::action::Dock;
  using GoalHandleDock = rclcpp_action::ClientGoalHandle<Dock>;

  explicit DockActionClient(const rclcpp::NodeOptions & options)
  : Node("execute_dock", options)
  {
    YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
    namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

    this->client_ptr_ = rclcpp_action::create_client<Dock>(
      this, "/" + namespace_ + "/" + std::string("autonomy/dock_local"));

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&DockActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "%s/autonomy/dock_local action server not available after waiting", namespace_.c_str());
      rclcpp::shutdown();
    }

    auto goal_msg = Dock::Goal();
    goal_msg.dock_name = DOCK_NAME;

    auto send_goal_options = rclcpp_action::Client<Dock>::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](const GoalHandleDock::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Dock Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Dock Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](
      GoalHandleDock::SharedPtr,
      const std::shared_ptr<const Dock::Feedback> feedback)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "Docking has been running for: %.3f", feedback->elapsed_time);
    };

    send_goal_options.result_callback = [this](const GoalHandleDock::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          if (result.result->success)
          {
            RCLCPP_INFO(this->get_logger(), "[Dock Goal Result] Succeeded! Result: Robot docked successfully!");
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "[Dock Goal Result] Succeeded! Result: Robot failed to dock");
          }
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "[Dock Goal Result] Aborted!");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "[Dock Goal Result] Cancelled!");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "[Dock Goal Result] Finished with unknown code");
          return;
      }
      rclcpp::shutdown();
    };

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Dock>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string namespace_;

};  // class DockActionClient

}  // namespace dock_action_client

int main()
{
}

RCLCPP_COMPONENTS_REGISTER_NODE(dock_action_client::DockActionClient)
