#include <functional>
#include <future>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include "clearpath_task_msgs/action/execute_task.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// TODO: Enter your task uuid
const std::string TASK_ID = "";


namespace task_action_client
{
class ExecuteTaskActionClient : public rclcpp::Node
{
public:
  using ExecuteTask = clearpath_task_msgs::action::ExecuteTask;
  using GoalHandleExecuteTask = rclcpp_action::ClientGoalHandle<ExecuteTask>;

  explicit ExecuteTaskActionClient(const rclcpp::NodeOptions & options)
  : Node("execute_task", options)
  {
    YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
    namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

    this->client_ptr_ = rclcpp_action::create_client<ExecuteTask>(
      this, "/" + namespace_ + "/" + std::string("execute_task"));

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ExecuteTaskActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "%s/execute_task action server not available after waiting", namespace_.c_str());
      rclcpp::shutdown();
    }

    auto goal_msg = ExecuteTask::Goal();
    goal_msg.task_id = TASK_ID;

    auto send_goal_options = rclcpp_action::Client<ExecuteTask>::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](const GoalHandleExecuteTask::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Task Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Task Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](
      GoalHandleExecuteTask::SharedPtr,
      const std::shared_ptr<const ExecuteTask::Feedback> feedback)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "Task has been running for: %.3f", feedback->elapsed_time);
    };

    send_goal_options.result_callback = [this](const GoalHandleExecuteTask::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          if (result.result->success)
          {
            RCLCPP_INFO(this->get_logger(), "[Task Goal Result] Succeeded! Result: Task executed successfully!");
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "[Taskl Result] Succeeded! Result: Failed to execute task");
          }
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "[Task Goal Result] Aborted!");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "[Task Goal Result] Cancelled!");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "[Task Goal Result] Finished with unknown code");
          return;
      }
      rclcpp::shutdown();
    };

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ExecuteTask>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string namespace_;

};  // class ExecuteTaskActionClient

}  // namespace task_action_client

int main()
{
}

RCLCPP_COMPONENTS_REGISTER_NODE(task_action_client::ExecuteTaskActionClient)
