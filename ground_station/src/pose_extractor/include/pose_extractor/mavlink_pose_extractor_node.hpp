#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "pose_extractor/mavlink_frame_parser.hpp"

namespace mavlink_pose_extractor {

/// ROS 2 node that extracts drone pose from raw MAVLink frames.
///
/// Subscribes to /drone_N/mavlink/from_fc (std_msgs/UInt8MultiArray) and
/// parses LOCAL_POSITION_NED (msgid=32) + ATTITUDE (msgid=30) messages.
/// Publishes /drone_N/pose/estimated as geometry_msgs/PoseStamped (ENU frame)
/// and broadcasts TF map → drone_N/base_link.
class MavlinkPoseExtractorNode : public rclcpp::Node {
 public:
  explicit MavlinkPoseExtractorNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void mavlink_callback(const std_msgs::msg::UInt8MultiArray& msg);
  void publish_timer_callback();

  // DESIGN: boot_time_ms from MAVLink is mapped to ROS clock by recording
  // the offset between ROS time and boot time on the first message received.
  // This avoids the need for external time synchronization.
  rclcpp::Time timestamp_from_boot_ms(uint32_t boot_ms);

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::optional<LocalPositionNed> latest_position_;
  std::optional<Attitude> latest_attitude_;
  std::string child_frame_id_;

  bool boot_offset_initialized_{false};
  int64_t boot_offset_ns_{0};
};

}  // namespace mavlink_pose_extractor
