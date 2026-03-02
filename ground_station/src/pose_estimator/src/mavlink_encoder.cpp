#include "pose_estimator/mavlink_encoder.hpp"

#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace pose_estimator {

void quaternion_to_euler(double qx, double qy, double qz, double qw,
                         float& roll, float& pitch, float& yaw) {
  // Roll (rotation around X axis)
  double sinr = 2.0 * (qw * qx + qy * qz);
  double cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
  roll = static_cast<float>(std::atan2(sinr, cosr));

  // Pitch (rotation around Y axis) — clamp at ±π/2 for gimbal-lock safety
  double sinp = 2.0 * (qw * qy - qz * qx);
  if (std::abs(sinp) >= 1.0) {
    pitch = static_cast<float>(std::copysign(M_PI / 2.0, sinp));
  } else {
    pitch = static_cast<float>(std::asin(sinp));
  }

  // Yaw (rotation around Z axis)
  double siny = 2.0 * (qw * qz + qx * qy);
  double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
  yaw = static_cast<float>(std::atan2(siny, cosy));
}

mavros_msgs::msg::Mavlink encode_vision_position_estimate(
    const geometry_msgs::msg::PoseStamped& pose, uint8_t seq, uint8_t sysid,
    uint8_t compid) {
  mavros_msgs::msg::Mavlink msg;
  msg.header = pose.header;
  msg.framing_status = mavros_msgs::msg::Mavlink::FRAMING_OK;
  msg.magic = kMavlink1Magic;
  msg.len = kVisionPosPayloadLen;
  msg.incompat_flags = 0;
  msg.compat_flags = 0;
  msg.seq = seq;
  msg.sysid = sysid;
  msg.compid = compid;
  msg.msgid = kMsgIdVisionPositionEstimate;

  // Timestamp: convert ROS stamp to microseconds for MAVLink
  uint64_t usec =
      static_cast<uint64_t>(pose.header.stamp.sec) * 1000000ULL +
      static_cast<uint64_t>(pose.header.stamp.nanosec) / 1000ULL;

  auto x = static_cast<float>(pose.pose.position.x);
  auto y = static_cast<float>(pose.pose.position.y);
  auto z = static_cast<float>(pose.pose.position.z);

  float roll = 0.0F;
  float pitch = 0.0F;
  float yaw = 0.0F;
  quaternion_to_euler(pose.pose.orientation.x, pose.pose.orientation.y,
                      pose.pose.orientation.z, pose.pose.orientation.w, roll,
                      pitch, yaw);

  // Pack 32-byte payload into 4 × uint64 (little-endian wire format):
  // [0..7] usec  [8..11] x  [12..15] y  [16..19] z
  // [20..23] roll  [24..27] pitch  [28..31] yaw
  uint8_t bytes[kVisionPosPayloadLen] = {};
  std::memcpy(bytes, &usec, sizeof(usec));
  std::memcpy(bytes + 8U, &x, sizeof(x));
  std::memcpy(bytes + 12U, &y, sizeof(y));
  std::memcpy(bytes + 16U, &z, sizeof(z));
  std::memcpy(bytes + 20U, &roll, sizeof(roll));
  std::memcpy(bytes + 24U, &pitch, sizeof(pitch));
  std::memcpy(bytes + 28U, &yaw, sizeof(yaw));

  msg.payload64.resize(kVisionPosPayloadLen / 8U);
  for (size_t i = 0; i < msg.payload64.size(); ++i) {
    std::memcpy(&msg.payload64[i], bytes + i * sizeof(uint64_t), sizeof(uint64_t));
  }

  return msg;
}

}  // namespace pose_estimator
