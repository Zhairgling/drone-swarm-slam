#include "pose_estimator/pose_estimator_node.hpp"

#include <functional>
#include <string>

#include "pose_estimator/mavlink_encoder.hpp"

namespace pose_estimator {

PoseEstimatorNode::PoseEstimatorNode(const rclcpp::NodeOptions& options)
    : Node("pose_estimator", options) {
  declare_parameter("drone_id", 1);
  declare_parameter("sysid", 1);
  declare_parameter("compid", 195);

  int drone_id = get_parameter("drone_id").as_int();
  sysid_ = static_cast<uint8_t>(get_parameter("sysid").as_int());
  compid_ = static_cast<uint8_t>(get_parameter("compid").as_int());

  std::string sub_topic =
      "/drone_" + std::to_string(drone_id) + "/pose/estimated";
  std::string pub_topic =
      "/drone_" + std::to_string(drone_id) + "/mavlink/to_fc";

  // DESIGN: best_effort for pose input (SLAM output is sensor-like, lossy ok).
  // Reliable for MAVLink output (flight controller corrections must arrive).
  auto sub_qos =
      rclcpp::QoS(rclcpp::KeepLast(10))
          .reliability(rclcpp::ReliabilityPolicy::BestEffort);
  auto pub_qos =
      rclcpp::QoS(rclcpp::KeepLast(10))
          .reliability(rclcpp::ReliabilityPolicy::Reliable);

  sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      sub_topic, sub_qos,
      std::bind(&PoseEstimatorNode::pose_callback, this,
                std::placeholders::_1));

  pub_ = create_publisher<mavros_msgs::msg::Mavlink>(pub_topic, pub_qos);

  RCLCPP_INFO(get_logger(),
              "PoseEstimator started: drone_id=%d, sub=%s, pub=%s", drone_id,
              sub_topic.c_str(), pub_topic.c_str());
}

void PoseEstimatorNode::pose_callback(
    geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  auto mavlink_msg =
      encode_vision_position_estimate(*msg, seq_++, sysid_, compid_);
  pub_->publish(mavlink_msg);
}

}  // namespace pose_estimator
