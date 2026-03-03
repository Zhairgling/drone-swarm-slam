#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "battery_monitor/battery_monitor_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<battery_monitor::BatteryMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
