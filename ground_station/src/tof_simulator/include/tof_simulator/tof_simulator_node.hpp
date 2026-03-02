#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "tof_simulator/tof_data_generator.hpp"

namespace tof_simulator {

/// ROS 2 node that publishes simulated VL53L8CX ToF data as PointCloud2.
///
/// Publishes on /drone_N/tof/pointcloud at 15Hz (matching real hardware rate).
/// All parameters are configurable via ROS 2 parameter system.
class TofSimulatorNode : public rclcpp::Node {
 public:
  explicit TofSimulatorNode(const rclcpp::NodeOptions& options =
                                rclcpp::NodeOptions());

 private:
  void timer_callback();

  std::unique_ptr<TofDataGenerator> generator_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
};

}  // namespace tof_simulator
