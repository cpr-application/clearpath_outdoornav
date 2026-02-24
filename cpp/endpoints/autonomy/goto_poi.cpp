#include <functional>
#include <future>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include "clearpath_navigation_msgs/action/execute_go_to_poi.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// TODO: Enter your map uuid and your point of interest uuid
const std::string MAP_ID = "";
const std::string POI_ID = "";


namespace goto_poi_action_client
{
class ExecuteGoToPOIActionClient : public rclcpp::Node
{
public:
  using ExecuteGoToPOI = clearpath_navigation_msgs::action::ExecuteGoToPOI;
  using GoalHandleExecuteGoToPOI = rclcpp_action::ClientGoalHandle<ExecuteGoToPOI>;

  explicit ExecuteGoToPOIActionClient(const rclcpp::NodeOptions & options)
  : Node("execute_goto_poi", options)
  {
    YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
    namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

    this->client_ptr_ = rclcpp_action::create_client<ExecuteGoToPOI>(
      this, "/" + namespace_ + "/" + std::string("autonomy/goto_poi"));

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ExecuteGoToPOIActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "%s/autonomy/goto_poi action server not available after waiting", namespace_.c_str());
      rclcpp::shutdown();
    }

    auto goal_msg = ExecuteGoToPOI::Goal();
    goal_msg.map_uuid = MAP_ID;
    goal_msg.poi_uuid = POI_ID;

    auto send_goal_options = rclcpp_action::Client<ExecuteGoToPOI>::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](const GoalHandleExecuteGoToPOI::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "GoTo POI Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "GoTo POI Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](
      GoalHandleExecuteGoToPOI::SharedPtr,
      const std::shared_ptr<const ExecuteGoToPOI::Feedback> feedback)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "GoTo has been running for: %.3f", feedback->elapsed_time);
    };

    send_goal_options.result_callback = [this](const GoalHandleExecuteGoToPOI::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          if (result.result->success)
          {
            RCLCPP_INFO(this->get_logger(), "[GoTo POI Goal Result] Succeeded! Result: Robot arrived at POI successfully!");
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "[GoTo POI Goal Result] Succeeded! Result: Robot failed to arrive to POI");
          }
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "[GoTo POI Goal Result] Aborted!");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "[GoTo POI Goal Result] Cancelled!");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "[GoTo POI Goal Result] Finished with unknown code");
          return;
      }
      rclcpp::shutdown();
    };

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ExecuteGoToPOI>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string namespace_;

};  // class ExecuteGoToPOIActionClient

}  // namespace goto_poi_action_client

int main()
{
}

RCLCPP_COMPONENTS_REGISTER_NODE(goto_poi_action_client::ExecuteGoToPOIActionClient)
