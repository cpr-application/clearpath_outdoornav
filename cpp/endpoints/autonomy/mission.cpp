#include <functional>
#include <future>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include "clearpath_navigation_msgs/action/execute_mission.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// TODO: Enter your mission and map uuids
const std::string MISSION_ID = "";
const std::string MAP_ID = "";

namespace mission_action_client
{
class ExecuteMissionActionClient : public rclcpp::Node
{
public:
  using ExecuteMission = clearpath_navigation_msgs::action::ExecuteMission;
  using GoalHandleExecuteMission = rclcpp_action::ClientGoalHandle<ExecuteMission>;

  explicit ExecuteMissionActionClient(const rclcpp::NodeOptions & options)
  : Node("execute_mission", options)
  {
    YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
    namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

    this->client_ptr_ = rclcpp_action::create_client<ExecuteMission>(
      this, "/" + namespace_ + "/" + std::string("autonomy/mission"));

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ExecuteMissionActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "%s/autonomy/mission Action server not available after waiting", namespace_.c_str());
      rclcpp::shutdown();
    }

    auto goal_msg = ExecuteMission::Goal();
    goal_msg.mission_uuid = MISSION_ID;
    goal_msg.map_uuid = MAP_ID;

    auto send_goal_options = rclcpp_action::Client<ExecuteMission>::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](const GoalHandleExecuteMission::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Mission Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Mission Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](
      GoalHandleExecuteMission::SharedPtr,
      const std::shared_ptr<const ExecuteMission::Feedback> feedback)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "Mission has been running for: %.3f", feedback->elapsed_time);
    };

    send_goal_options.result_callback = [this](const GoalHandleExecuteMission::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          if (result.result->success)
          {
            RCLCPP_INFO(this->get_logger(), "[Mission Goal Result] Succeeded! Result: Robot completed mission!");
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "[Mission Goal Result] Succeeded! Result: Robot failed to complete mission");
          }
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "[Mission Goal Result] Aborted!");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "[Mission Goal Result] Cancelled!");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "[Mission Goal Result] Finished with unknown code");
          return;
      }
      rclcpp::shutdown();
    };

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ExecuteMission>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string namespace_;

};  // class ExecuteMissionActionClient

}  // namespace mission_action_client

int main()
{
}

RCLCPP_COMPONENTS_REGISTER_NODE(mission_action_client::ExecuteMissionActionClient)
