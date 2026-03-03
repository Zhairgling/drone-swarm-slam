#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace battery_monitor {

/// ROS 2 node that extracts battery state from raw MAVLink SYS_STATUS frames.
///
/// Subscribes to /drone_N/mavlink/from_fc (std_msgs/UInt8MultiArray), parses
/// SYS_STATUS (msgid=1), and publishes /drone_N/battery as
/// sensor_msgs/BatteryState. Logs warnings at configurable voltage thresholds.
class BatteryMonitorNode : public rclcpp::Node {
 public:
  explicit BatteryMonitorNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void mavlink_callback(const std_msgs::msg::UInt8MultiArray& msg);

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_;

  double warn_voltage_;
  double critical_voltage_;
};

}  // namespace battery_monitor
