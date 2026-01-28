#include <functional>
#include <future>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include "clearpath_navigation_msgs/action/execute_mission_from_goal.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// TODO: Enter your mission, map and starting goal uuids
const std::string MISSION_ID = "";
const std::string MAP_ID = "";
const std::string GOAL_ID = "";
const bool RUN_ON_START_TASKS = true;

namespace mission_from_goal_action_client
{
class ExecuteMissionFromGoalActionClient : public rclcpp::Node
{
public:
  using ExecuteMissionFromGoal = clearpath_navigation_msgs::action::ExecuteMissionFromGoal;
  using GoalHandleExecuteMissionFromGoal = rclcpp_action::ClientGoalHandle<ExecuteMissionFromGoal>;

  explicit ExecuteMissionFromGoalActionClient(const rclcpp::NodeOptions & options)
  : Node("execute_mission_from_goal", options)
  {
    YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
    namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

    this->client_ptr_ = rclcpp_action::create_client<ExecuteMissionFromGoal>(
      this, "/" + namespace_ + "/" + std::string("autonomy/mission_from_goal"));

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ExecuteMissionFromGoalActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "%s/autonomy/mission_from_goal Action server not available after waiting", namespace_.c_str());
      rclcpp::shutdown();
    }

    auto goal_msg = ExecuteMissionFromGoal::Goal();
    goal_msg.mission_uuid = MISSION_ID;
    goal_msg.map_uuid = MAP_ID;
    goal_msg.goal_uuid = GOAL_ID;
    goal_msg.run_on_start_tasks = RUN_ON_START_TASKS;

    auto send_goal_options = rclcpp_action::Client<ExecuteMissionFromGoal>::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](const GoalHandleExecuteMissionFromGoal::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Mission From Goal Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Mission From Goal Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](
      GoalHandleExecuteMissionFromGoal::SharedPtr,
      const std::shared_ptr<const ExecuteMissionFromGoal::Feedback> feedback)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "Mission has been running for: %.3f", feedback->elapsed_time);
    };

    send_goal_options.result_callback = [this](const GoalHandleExecuteMissionFromGoal::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          if (result.result->success)
          {
            RCLCPP_INFO(this->get_logger(), "[Mission from Goal Goal Result] Succeeded! Result: Robot completed mission!");
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "[Mission from Goal Goal Result] Succeeded! Result: Robot failed to complete mission");
          }
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "[Mission from Goal Goal Result] Aborted!");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "[Mission from Goal Goal Result] Cancelled!");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "[Mission from Goal Goal Result] Finished with unknown code");
          return;
      }
      rclcpp::shutdown();
    };

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ExecuteMissionFromGoal>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string namespace_;

};  // class ExecuteMissionFromGoalActionClient

}  // namespace mission_from_goal_action_client

int main()
{
}

RCLCPP_COMPONENTS_REGISTER_NODE(mission_from_goal_action_client::ExecuteMissionFromGoalActionClient)
