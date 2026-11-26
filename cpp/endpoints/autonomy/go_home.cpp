// BETA: The go home API is in beta and will be available in the next release.

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include "clearpath_navigation_msgs/action/execute_go_home.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// TODO: Enter the uuid of the map to navigate home on
// The home location itself is set with the mission_manager/set_home service
const std::string MAP_ID = "";


namespace go_home_action_client
{
class ExecuteGoHomeActionClient : public rclcpp::Node
{
public:
  using ExecuteGoHome = clearpath_navigation_msgs::action::ExecuteGoHome;
  using GoalHandleExecuteGoHome = rclcpp_action::ClientGoalHandle<ExecuteGoHome>;

  explicit ExecuteGoHomeActionClient(const rclcpp::NodeOptions & options)
  : Node("execute_go_home", options)
  {
    YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
    namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

    this->client_ptr_ = rclcpp_action::create_client<ExecuteGoHome>(
      this, "/" + namespace_ + "/" + std::string("autonomy/go_home"));

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ExecuteGoHomeActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "%s/autonomy/go_home action server not available after waiting", namespace_.c_str());
      rclcpp::shutdown();
    }

    auto goal_msg = ExecuteGoHome::Goal();
    goal_msg.map_uuid = MAP_ID;

    auto send_goal_options = rclcpp_action::Client<ExecuteGoHome>::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](const GoalHandleExecuteGoHome::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Go Home Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Go Home Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](
      GoalHandleExecuteGoHome::SharedPtr,
      const std::shared_ptr<const ExecuteGoHome::Feedback> feedback)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "Go Home has been running for: %.3f", feedback->elapsed_time);
    };

    send_goal_options.result_callback = [this](const GoalHandleExecuteGoHome::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          if (result.result->success)
          {
            RCLCPP_INFO(this->get_logger(), "[Go Home Goal Result] Succeeded! Result: Robot is home!");
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "[Go Home Goal Result] Robot failed to go home: %s",
                         result.result->error_msg.c_str());
          }
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "[Go Home Goal Result] Aborted!");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "[Go Home Goal Result] Cancelled!");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "[Go Home Goal Result] Finished with unknown code");
          break;
      }
      rclcpp::shutdown();
    };

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ExecuteGoHome>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string namespace_;

};  // class ExecuteGoHomeActionClient

}  // namespace go_home_action_client

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go_home_action_client::ExecuteGoHomeActionClient>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(go_home_action_client::ExecuteGoHomeActionClient)
