#pragma once

#include <cstdint>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/mavlink.hpp>
#include <rclcpp/rclcpp.hpp>

namespace pose_estimator {

/// ROS 2 node that converts SLAM-estimated pose to MAVLink corrections.
///
/// Subscribes to /drone_N/pose/estimated (geometry_msgs/PoseStamped) and
/// publishes VISION_POSITION_ESTIMATE MAVLink frames on /drone_N/mavlink/to_fc
/// for ArduCopter's EKF to use as an external vision position source.
class PoseEstimatorNode : public rclcpp::Node {
 public:
  explicit PoseEstimatorNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void pose_callback(
      const geometry_msgs::msg::PoseStamped::SharedPtr& msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<mavros_msgs::msg::Mavlink>::SharedPtr pub_;

  // DESIGN: seq_ wraps from 255 → 0, matching MAVLink sequence semantics.
  uint8_t seq_{0};
  uint8_t sysid_{1};
  // DESIGN: compid 195 = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY (MAVLink spec).
  uint8_t compid_{195};
};

}  // namespace pose_estimator
