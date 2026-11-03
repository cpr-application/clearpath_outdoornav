#include "rclcpp/rclcpp.hpp"
#include "clearpath_dock_msgs/msg/dock_status.hpp"

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
using DockStatus = clearpath_dock_msgs::msg::DockStatus;

namespace
{
// The monitor publishes at 10 Hz; treat a longer silence as the monitor being unavailable.
constexpr auto kStatusTimeout = 1s;

std::string stateName(uint8_t state)
{
  switch (state)
  {
    case DockStatus::UNKNOWN: return "UNKNOWN";
    case DockStatus::UNDOCKED: return "UNDOCKED";
    case DockStatus::DOCKED: return "DOCKED";
    default: return std::to_string(state);
  }
}

std::string chargingName(uint8_t state)
{
  switch (state)
  {
    case DockStatus::CHARGE_UNKNOWN: return "UNKNOWN";
    case DockStatus::NOT_CHARGING: return "NOT_CHARGING";
    case DockStatus::CHARGING: return "CHARGING";
    case DockStatus::FULL: return "FULL";
    default: return std::to_string(state);
  }
}

std::string evidenceName(uint8_t evidence)
{
  switch (evidence)
  {
    case DockStatus::EVIDENCE_NONE: return "none";
    case DockStatus::EVIDENCE_GEOMETRY: return "geometry";
    case DockStatus::EVIDENCE_HARDWARE: return "hardware";
    case DockStatus::EVIDENCE_BOTH: return "geometry+hardware";
    default: return std::to_string(evidence);
  }
}
}  // namespace

class DockStatusMonitor : public rclcpp::Node
{
public:
  explicit DockStatusMonitor(const std::string& namespace_) : Node("dock_status_monitor")
  {
    const std::string topic = namespace_.empty() ? "docking/status" : "/" + namespace_ + "/docking/status";

    // Transient-local delivers the latest status immediately on connection.
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    subscription_ = create_subscription<DockStatus>(
        topic, qos, [this](const DockStatus& msg) { statusCallback(msg); });

    RCLCPP_INFO(get_logger(), "Monitoring %s", topic.c_str());
    output_timer_ = create_wall_timer(5s, [this]() { outputStatus(); });
    liveness_timer_ = create_wall_timer(500ms, [this]() { checkLiveness(); });
  }

private:
  void statusCallback(const DockStatus& msg)
  {
    last_status_ = msg;
    last_received_ = std::chrono::steady_clock::now();
    if (stale_reported_)
    {
      RCLCPP_INFO(get_logger(), "Dock status stream resumed");
      stale_reported_ = false;
    }

    const auto key = std::make_tuple(msg.state, msg.charging_state, msg.conflict, msg.charger_fault);
    if (!last_key_ || *last_key_ != key)
    {
      last_key_ = key;
      logStatus(msg, true);
    }
  }

  void checkLiveness()
  {
    if (!last_received_ || stale_reported_)
    {
      return;
    }
    const auto age = std::chrono::steady_clock::now() - *last_received_;
    if (age > kStatusTimeout)
    {
      stale_reported_ = true;
      RCLCPP_WARN(get_logger(), "No dock status for %.1fs; treat physical dock state as UNKNOWN",
                  std::chrono::duration<double>(age).count());
    }
  }

  void outputStatus()
  {
    if (!last_status_)
    {
      RCLCPP_INFO(get_logger(), "Waiting for dock status...");
    }
    else if (!stale_reported_)
    {
      logStatus(*last_status_, false);
    }
  }

  void logStatus(const DockStatus& msg, bool changed)
  {
    const char* prefix = changed ? "Dock status changed" : "Dock status";
    const std::string text = std::string(prefix) + ": state = " + stateName(msg.state) +
                             ", charging = " + chargingName(msg.charging_state) +
                             ", evidence = " + evidenceName(msg.evidence) +
                             ", template = " + (msg.dock_template.empty() ? "n/a" : msg.dock_template) +
                             ", conflict = " + (msg.conflict ? "true" : "false") +
                             ", charger_fault = " + (msg.charger_fault ? "true" : "false") +
                             ", reason = \"" + msg.reason + "\"";
    if (msg.conflict || msg.charger_fault)
    {
      RCLCPP_WARN(get_logger(), "%s", text.c_str());
    }
    else
    {
      RCLCPP_INFO(get_logger(), "%s", text.c_str());
    }
    if (msg.geometry_fresh)
    {
      RCLCPP_INFO(get_logger(), "Pose error from docked pose: x = %.3f m, y = %.3f m, yaw = %.3f rad",
                  msg.pose_error.x, msg.pose_error.y, msg.pose_error.theta);
    }
  }

  rclcpp::Subscription<DockStatus>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr output_timer_;
  rclcpp::TimerBase::SharedPtr liveness_timer_;
  std::optional<DockStatus> last_status_;
  std::optional<std::tuple<uint8_t, uint8_t, bool, bool>> last_key_;
  std::optional<std::chrono::steady_clock::time_point> last_received_;
  bool stale_reported_{ false };
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  YAML::Node robot_config = YAML::LoadFile("/etc/clearpath/robot.yaml");
  std::string namespace_ = robot_config["system"]["ros2"]["namespace"].as<std::string>();

  rclcpp::spin(std::make_shared<DockStatusMonitor>(namespace_));
  rclcpp::shutdown();
  return 0;
}
