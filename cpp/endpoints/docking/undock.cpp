#include <functional>
#include <future>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include "clearpath_dock_msgs/action/undock.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// TODO: Enter your dock name
const std::string DOCK_NAME = "";

namespace undock_action_client
{
class UndockActionClient : public rclcpp::Node
{
public:
  using Undock = clearpath_dock_msgs::action::Undock;
  using GoalHandleUndock = rclcpp_action::ClientGoalHandle<Undock>;

  explicit UndockActionClient(const rclcpp::NodeOptions & options)
  : Node("execute_undock", options)
  {
    YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
    namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

    this->client_ptr_ = rclcpp_action::create_client<Undock>(
      this, "/" + namespace_ + "/" + std::string("autonomy/undock"));

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&UndockActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "%s/autonomy/undock Action server not available after waiting", namespace_.c_str());
      rclcpp::shutdown();
    }

    auto goal_msg = Undock::Goal();
    goal_msg.dock_name = DOCK_NAME;

    auto send_goal_options = rclcpp_action::Client<Undock>::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](const GoalHandleUndock::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Undock Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Undock Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](
      GoalHandleUndock::SharedPtr,
      const std::shared_ptr<const Undock::Feedback> feedback)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "Undocking has been running for: %.3f", feedback->elapsed_time);
    };

    send_goal_options.result_callback = [this](const GoalHandleUndock::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          if (result.result->success)
          {
            RCLCPP_INFO(this->get_logger(), "[Undock Goal Result] Succeeded! Result: Robot undocked successfully!");
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "[Undock Goal Result] Succeeded! Result: Robot failed to undock");
          }
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "[Undock Goal Result] Aborted!");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "[Undock Goal Result] Cancelled!");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "[Undock Goal Result] Finished with unknown code");
          return;
      }
      rclcpp::shutdown();
    };

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Undock>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string namespace_;

};  // class UndockActionClient

}  // namespace undock_action_client

int main()
{
}

RCLCPP_COMPONENTS_REGISTER_NODE(undock_action_client::UndockActionClient)
