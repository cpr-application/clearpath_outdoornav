#include <functional>
#include <future>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include "clearpath_navigation_msgs/action/execute_go_to.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// TODO: Enter your map id and waypoint information
const std::string MAP_ID = "";
const std::string WAYPOINT_ID = "";
const std::string WAYPOINT_NAME = "";
const double WAYPOINT_LAT = ;
const double WAYPOINT_LON = ;
const double WAYPOINT_HEADING = ;
const double WAYPOINT_POSITION_TOLERANCE = -1.0;  // -1.0 means off
const double WAYPOINT_YAW_TOLERANCE = -1.0;       // -1.0 means off


namespace goto_action_client
{
class ExecuteGoToActionClient : public rclcpp::Node
{
public:
  using ExecuteGoTo = clearpath_navigation_msgs::action::ExecuteGoTo;
  using GoalHandleExecuteGoTo = rclcpp_action::ClientGoalHandle<ExecuteGoTo>;

  explicit ExecuteGoToActionClient(const rclcpp::NodeOptions & options)
  : Node("execute_goto", options)
  {
    YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
    namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

    this->client_ptr_ = rclcpp_action::create_client<ExecuteGoTo>(
      this, "/" + namespace_ + "/" + std::string("autonomy/goto"));

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ExecuteGoToActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "%s/autonomy/goto action server not available after waiting", namespace_.c_str());
      rclcpp::shutdown();
    }

    auto goal_msg = ExecuteGoTo::Goal();
    goal_msg.map_uuid = MAP_ID;
    
    auto waypoint = clearpath_navigation_msgs::msg::Waypoint();
    waypoint.uuid = WAYPOINT_ID;
    waypoint.name = WAYPOINT_NAME;
    waypoint.uuid = WAYPOINT_ID;
    waypoint.latitude = WAYPOINT_LAT;
    waypoint.longitude = WAYPOINT_LON;
    waypoint.heading = WAYPOINT_HEADING;
    waypoint.position_tolerance = WAYPOINT_POSITION_TOLERANCE;
    waypoint.yaw_tolerance = WAYPOINT_YAW_TOLERANCE;
    goal_msg.waypoint = waypoint;

    auto send_goal_options = rclcpp_action::Client<ExecuteGoTo>::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](const GoalHandleExecuteGoTo::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "GoTo Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "GoTo Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](
      GoalHandleExecuteGoTo::SharedPtr,
      const std::shared_ptr<const ExecuteGoTo::Feedback> feedback)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "GoTo has been running for: %.3f", feedback->elapsed_time);
    };

    send_goal_options.result_callback = [this](const GoalHandleExecuteGoTo::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          if (result.result->success)
          {
            RCLCPP_INFO(this->get_logger(), "[GoTo Goal Result] Succeeded! Result: Robot arrived at goto position successfully!");
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "[GoTo Goal Result] Succeeded! Result: Robot failed to arrive to goto position");
          }
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "[GoTo Goal Result] Aborted!");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "[GoTo Goal Result] Cancelled!");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "[GoTo Goal Result] Finished with unknown code");
          return;
      }
      rclcpp::shutdown();
    };

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ExecuteGoTo>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string namespace_;

};  // class ExecuteGoToActionClient

}  // namespace goto_action_client

int main()
{
}

RCLCPP_COMPONENTS_REGISTER_NODE(goto_action_client::ExecuteGoToActionClient)
