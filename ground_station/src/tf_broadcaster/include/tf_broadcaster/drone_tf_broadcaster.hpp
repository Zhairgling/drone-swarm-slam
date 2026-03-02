#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace tf_broadcaster {

// DESIGN: 5cm offset matches tof_simulator default_sensor_poses() so that
// the TF tree is consistent with simulated pointcloud origins.
constexpr double kDefaultSensorOffset = 0.05;  // 5cm from drone center

struct SensorOffset {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;  // radians, rotation around Z-axis (FLU frame)
};

struct TfConfig {
  int drone_id = 1;
  SensorOffset tof_front;
  SensorOffset tof_right;
  SensorOffset tof_back;
  SensorOffset tof_left;
  SensorOffset camera;
};

/// Returns defaults consistent with tof_simulator default_sensor_poses().
/// FLU frame: +X=forward, +Y=left, +Z=up.
TfConfig default_config();

/// Build one TransformStamped from parent_frame to child_frame.
/// Uses zero timestamp (appropriate for static transforms).
geometry_msgs::msg::TransformStamped make_transform(
    const std::string& parent_frame, const std::string& child_frame,
    const SensorOffset& offset);

/// Build all 5 static transforms (4 ToF + 1 camera) for the given config.
std::vector<geometry_msgs::msg::TransformStamped> build_transforms(
    const TfConfig& config);

class DroneTfBroadcaster : public rclcpp::Node {
 public:
  explicit DroneTfBroadcaster(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

}  // namespace tf_broadcaster
