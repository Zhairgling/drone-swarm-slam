#include "pose_extractor/mavlink_pose_extractor_node.hpp"

#include <chrono>
#include <functional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace mavlink_pose_extractor {

MavlinkPoseExtractorNode::MavlinkPoseExtractorNode(
    const rclcpp::NodeOptions& options)
    : Node("mavlink_pose_extractor", options) {
  declare_parameter("drone_id", 1);
  declare_parameter("publish_rate_hz", 30.0);

  const int drone_id = get_parameter("drone_id").as_int();
  const double rate_hz = get_parameter("publish_rate_hz").as_double();

  const std::string ns = "/drone_" + std::to_string(drone_id);
  const std::string sub_topic = ns + "/mavlink/from_fc";
  const std::string pub_topic = ns + "/pose/estimated";
  child_frame_id_ = "drone_" + std::to_string(drone_id) + "/base_link";

  // DESIGN: Reliable QoS for MAVLink — same as firmware bridge uses on from_fc.
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                 .reliability(rclcpp::ReliabilityPolicy::Reliable);

  sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      sub_topic, qos,
      [this](const std_msgs::msg::UInt8MultiArray& msg) {
        mavlink_callback(msg);
      });

  // DESIGN: BestEffort for pose output — downstream SLAM consumers are
  // sensor-like and tolerate dropped frames.
  auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                     .reliability(rclcpp::ReliabilityPolicy::BestEffort);
  pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pub_topic, pub_qos);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  auto period = std::chrono::duration<double>(1.0 / rate_hz);
  publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MavlinkPoseExtractorNode::publish_timer_callback, this));

  RCLCPP_INFO(get_logger(),
              "MavlinkPoseExtractor started: drone_id=%d, sub=%s, pub=%s, "
              "rate=%.1f Hz",
              drone_id, sub_topic.c_str(), pub_topic.c_str(), rate_hz);
}

void MavlinkPoseExtractorNode::mavlink_callback(
    const std_msgs::msg::UInt8MultiArray& msg) {
  const std::vector<uint8_t>& frame = msg.data;

  LocalPositionNed pos{};
  Attitude att{};

  if (parse_local_position_ned(frame, pos)) {
    latest_position_ = pos;
  } else if (parse_attitude(frame, att)) {
    latest_attitude_ = att;
  }
}

void MavlinkPoseExtractorNode::publish_timer_callback() {
  if (!latest_position_.has_value() || !latest_attitude_.has_value()) return;

  const auto& pos = *latest_position_;
  const auto& att = *latest_attitude_;

  // Use the more recent of the two boot timestamps.
  const uint32_t boot_ms =
      (pos.time_boot_ms > att.time_boot_ms) ? pos.time_boot_ms
                                             : att.time_boot_ms;
  const rclcpp::Time stamp = timestamp_from_boot_ms(boot_ms);

  // NED → ENU position: (x_enu, y_enu, z_enu) = (y_ned, x_ned, -z_ned)
  const double x_enu = static_cast<double>(pos.y);
  const double y_enu = static_cast<double>(pos.x);
  const double z_enu = -static_cast<double>(pos.z);

  double qx{}, qy{}, qz{}, qw{};
  ned_attitude_to_enu_quaternion(att.roll, att.pitch, att.yaw, qx, qy, qz, qw);

  // Publish PoseStamped
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = "map";
  pose_msg.pose.position.x = x_enu;
  pose_msg.pose.position.y = y_enu;
  pose_msg.pose.position.z = z_enu;
  pose_msg.pose.orientation.x = qx;
  pose_msg.pose.orientation.y = qy;
  pose_msg.pose.orientation.z = qz;
  pose_msg.pose.orientation.w = qw;
  pub_->publish(pose_msg);

  // Broadcast TF: map → drone_N/base_link
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = child_frame_id_;
  tf_msg.transform.translation.x = x_enu;
  tf_msg.transform.translation.y = y_enu;
  tf_msg.transform.translation.z = z_enu;
  tf_msg.transform.rotation.x = qx;
  tf_msg.transform.rotation.y = qy;
  tf_msg.transform.rotation.z = qz;
  tf_msg.transform.rotation.w = qw;
  tf_broadcaster_->sendTransform(tf_msg);
}

rclcpp::Time MavlinkPoseExtractorNode::timestamp_from_boot_ms(
    uint32_t boot_ms) {
  const int64_t boot_ns = static_cast<int64_t>(boot_ms) * 1000000LL;
  if (!boot_offset_initialized_) {
    boot_offset_ns_ = this->now().nanoseconds() - boot_ns;
    boot_offset_initialized_ = true;
  }
  return rclcpp::Time(boot_offset_ns_ + boot_ns);
}

}  // namespace mavlink_pose_extractor
