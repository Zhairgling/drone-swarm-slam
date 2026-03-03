#include "battery_monitor/battery_monitor_node.hpp"

#include <cmath>
#include <string>

#include "battery_monitor/mavlink_battery_parser.hpp"

namespace battery_monitor {

BatteryMonitorNode::BatteryMonitorNode(const rclcpp::NodeOptions& options)
    : Node("battery_monitor", options) {
  declare_parameter("drone_id", 1);
  declare_parameter("warn_voltage", 14.2);
  declare_parameter("critical_voltage", 13.6);

  const int drone_id = get_parameter("drone_id").as_int();
  warn_voltage_ = get_parameter("warn_voltage").as_double();
  critical_voltage_ = get_parameter("critical_voltage").as_double();

  const std::string ns = "/drone_" + std::to_string(drone_id);
  const std::string sub_topic = ns + "/mavlink/from_fc";
  const std::string pub_topic = ns + "/battery";

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                 .reliability(rclcpp::ReliabilityPolicy::Reliable);

  sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      sub_topic, qos,
      [this](const std_msgs::msg::UInt8MultiArray& msg) {
        mavlink_callback(msg);
      });

  // DESIGN: BestEffort for battery state — consumers are monitoring-like
  // and tolerate occasional dropped messages.
  auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                     .reliability(rclcpp::ReliabilityPolicy::BestEffort);
  pub_ = create_publisher<sensor_msgs::msg::BatteryState>(pub_topic, pub_qos);

  RCLCPP_INFO(get_logger(),
              "BatteryMonitor started: drone_id=%d, sub=%s, pub=%s, "
              "warn=%.1fV, critical=%.1fV",
              drone_id, sub_topic.c_str(), pub_topic.c_str(), warn_voltage_,
              critical_voltage_);
}

void BatteryMonitorNode::mavlink_callback(
    const std_msgs::msg::UInt8MultiArray& msg) {
  SysStatus status{};
  if (!parse_sys_status(msg.data, status)) return;

  // Convert units: mV→V, cA→A, %→[0..1]
  const float voltage =
      (status.voltage_battery == 0xFFFFu)
          ? std::numeric_limits<float>::quiet_NaN()
          : static_cast<float>(status.voltage_battery) / 1000.0f;
  const float current =
      (status.current_battery == -1)
          ? std::numeric_limits<float>::quiet_NaN()
          : static_cast<float>(status.current_battery) / 100.0f;
  const float percentage =
      (status.battery_remaining == -1)
          ? std::numeric_limits<float>::quiet_NaN()
          : static_cast<float>(status.battery_remaining) / 100.0f;

  sensor_msgs::msg::BatteryState bat;
  bat.header.stamp = this->now();
  bat.voltage = voltage;
  bat.current = current;
  bat.percentage = percentage;
  bat.temperature = std::numeric_limits<float>::quiet_NaN();
  bat.charge = std::numeric_limits<float>::quiet_NaN();
  bat.capacity = std::numeric_limits<float>::quiet_NaN();
  bat.design_capacity = std::numeric_limits<float>::quiet_NaN();
  bat.present = true;
  bat.power_supply_status =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  bat.power_supply_health =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  // DESIGN: 4S LiPo chemistry
  bat.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
  pub_->publish(bat);

  // Threshold warnings — only if voltage is available
  if (!std::isnan(voltage)) {
    if (voltage <= critical_voltage_) {
      RCLCPP_ERROR(get_logger(),
                   "CRITICAL battery voltage: %.2fV (threshold %.1fV)",
                   voltage, critical_voltage_);
    } else if (voltage <= warn_voltage_) {
      RCLCPP_WARN(get_logger(),
                  "Low battery voltage: %.2fV (threshold %.1fV)", voltage,
                  warn_voltage_);
    }
  }
}

}  // namespace battery_monitor
