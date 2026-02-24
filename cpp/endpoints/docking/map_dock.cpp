#include <functional>
#include <future>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

#include "clearpath_dock_msgs/action/map_dock.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// TODO: Enter your dock name and map uuid
const std::string DOCK_NAME = "";
const std::string MAP_ID = "";

namespace map_dock_action_client
{
class MapDockActionClient : public rclcpp::Node
{
public:
  using MapDock = clearpath_dock_msgs::action::MapDock;
  using GoalHandleMapDock = rclcpp_action::ClientGoalHandle<MapDock>;

  explicit MapDockActionClient(const rclcpp::NodeOptions & options)
  : Node("execute_map_dock", options)
  {
    YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
    namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

    this->client_ptr_ = rclcpp_action::create_client<MapDock>(
      this, "/" + namespace_ + "/" + std::string("autonomy/dock_map"));

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MapDockActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "%s/autonomy/dock_map action server not available after waiting", namespace_.c_str());
      rclcpp::shutdown();
    }

    auto goal_msg = MapDock::Goal();
    goal_msg.dock_name = DOCK_NAME;
    goal_msg.map_uuid = MAP_ID;

    auto send_goal_options = rclcpp_action::Client<MapDock>::SendGoalOptions();

    send_goal_options.goal_response_callback = [this](const GoalHandleMapDock::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Map Dock Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Map Dock Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](
      GoalHandleMapDock::SharedPtr,
      const std::shared_ptr<const MapDock::Feedback> feedback)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "Map docking has been running for: %.3f", feedback->elapsed_time);
    };

    send_goal_options.result_callback = [this](const GoalHandleMapDock::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          if (result.result->success)
          {
            RCLCPP_INFO(this->get_logger(), "[Map Dock Goal Result] Succeeded! Result: Robot map docked successfully!");
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "[Map Dock Goal Result] Succeeded! Result: Robot failed to map dock");
          }
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "[Map Dock Goal Result] Aborted!");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "[Map Dock Goal Result] Cancelled!");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "[Map Dock Goal Result] Finished with unknown code");
          return;
      }
      rclcpp::shutdown();
    };

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<MapDock>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string namespace_;

};  // class MapDockActionClient

}  // namespace map_dock_action_client

int main()
{
}

RCLCPP_COMPONENTS_REGISTER_NODE(map_dock_action_client::MapDockActionClient)
