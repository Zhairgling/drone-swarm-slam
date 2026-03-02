#pragma once

#include <cstdint>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/mavlink.hpp>

namespace pose_estimator {

// DESIGN: VISION_POSITION_ESTIMATE (msgid=102) sends SLAM pose to ArduCopter
// for EKF correction. Payload: usec(8) + xyz(12) + rpy(12) = 32 bytes.
constexpr uint64_t kMsgIdVisionPositionEstimate = 102;
constexpr uint8_t kMavlink1Magic = 0xFE;
constexpr uint8_t kVisionPosPayloadLen = 32;

/// Convert a unit quaternion to Euler angles using ZYX convention.
///
/// ArduCopter's EKF expects ZYX (yaw-pitch-roll) Euler angles, which is
/// the standard aerospace convention: yaw around Z, pitch around Y, roll around X.
void quaternion_to_euler(double qx, double qy, double qz, double qw,
                         float& roll, float& pitch, float& yaw);

/// Encode a PoseStamped as a MAVLink VISION_POSITION_ESTIMATE message.
///
/// The returned mavros_msgs/Mavlink is ready to publish on
/// /drone_N/mavlink/to_fc for the MAVLink bridge to forward to ArduCopter.
mavros_msgs::msg::Mavlink encode_vision_position_estimate(
    const geometry_msgs::msg::PoseStamped& pose, uint8_t seq, uint8_t sysid,
    uint8_t compid);

}  // namespace pose_estimator
