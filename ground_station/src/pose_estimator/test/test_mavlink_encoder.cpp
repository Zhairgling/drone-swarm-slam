// cppcheck-suppress-file syntaxError
#include "pose_estimator/mavlink_encoder.hpp"

#include <cmath>
#include <cstring>

#include <gtest/gtest.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace pose_estimator {
namespace {

geometry_msgs::msg::PoseStamped make_pose(float x, float y, float z,
                                          double qx, double qy, double qz,
                                          double qw, int32_t sec = 1,
                                          uint32_t nanosec = 0) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp.sec = sec;
  pose.header.stamp.nanosec = nanosec;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.x = qx;
  pose.pose.orientation.y = qy;
  pose.pose.orientation.z = qz;
  pose.pose.orientation.w = qw;
  return pose;
}

/// Unpack the 32-byte VISION_POSITION_ESTIMATE payload from a Mavlink message.
struct DecodedPayload {
  uint64_t usec;
  float x, y, z, roll, pitch, yaw;
};

DecodedPayload decode_payload(const mavros_msgs::msg::Mavlink& msg) {
  uint8_t bytes[32] = {};
  for (size_t i = 0; i < msg.payload64.size() && i < 4U; ++i) {
    std::memcpy(bytes + i * sizeof(uint64_t), &msg.payload64[i],
                sizeof(uint64_t));
  }
  DecodedPayload p{};
  std::memcpy(&p.usec, bytes + 0, 8);
  std::memcpy(&p.x, bytes + 8, 4);
  std::memcpy(&p.y, bytes + 12, 4);
  std::memcpy(&p.z, bytes + 16, 4);
  std::memcpy(&p.roll, bytes + 20, 4);
  std::memcpy(&p.pitch, bytes + 24, 4);
  std::memcpy(&p.yaw, bytes + 28, 4);
  return p;
}

// --- quaternion_to_euler ---

TEST(QuaternionToEuler, IdentityGivesZeroAngles) {
  float roll = 1.0F;
  float pitch = 1.0F;
  float yaw = 1.0F;
  quaternion_to_euler(0.0, 0.0, 0.0, 1.0, roll, pitch, yaw);
  EXPECT_NEAR(roll, 0.0F, 1e-6F);
  EXPECT_NEAR(pitch, 0.0F, 1e-6F);
  EXPECT_NEAR(yaw, 0.0F, 1e-6F);
}

TEST(QuaternionToEuler, PureYaw90Degrees) {
  // 90° yaw: q = (0, 0, sin(π/4), cos(π/4))
  auto s = std::sin(M_PI / 4.0);
  auto c = std::cos(M_PI / 4.0);
  float roll = 0.0F;
  float pitch = 0.0F;
  float yaw = 0.0F;
  quaternion_to_euler(0.0, 0.0, s, c, roll, pitch, yaw);
  EXPECT_NEAR(roll, 0.0F, 1e-5F);
  EXPECT_NEAR(pitch, 0.0F, 1e-5F);
  EXPECT_NEAR(yaw, static_cast<float>(M_PI / 2.0), 1e-5F);
}

TEST(QuaternionToEuler, PureRoll90Degrees) {
  // 90° roll: q = (sin(π/4), 0, 0, cos(π/4))
  auto s = std::sin(M_PI / 4.0);
  auto c = std::cos(M_PI / 4.0);
  float roll = 0.0F;
  float pitch = 0.0F;
  float yaw = 0.0F;
  quaternion_to_euler(s, 0.0, 0.0, c, roll, pitch, yaw);
  EXPECT_NEAR(roll, static_cast<float>(M_PI / 2.0), 1e-5F);
  EXPECT_NEAR(pitch, 0.0F, 1e-5F);
  EXPECT_NEAR(yaw, 0.0F, 1e-5F);
}

TEST(QuaternionToEuler, NegativeYaw) {
  // -90° yaw
  auto s = std::sin(-M_PI / 4.0);
  auto c = std::cos(M_PI / 4.0);
  float roll = 0.0F;
  float pitch = 0.0F;
  float yaw = 0.0F;
  quaternion_to_euler(0.0, 0.0, s, c, roll, pitch, yaw);
  EXPECT_NEAR(yaw, static_cast<float>(-M_PI / 2.0), 1e-5F);
}

TEST(QuaternionToEuler, GimbalLockPitchClamped) {
  // Near +90° pitch: sinp → 1, pitch should be clamped to π/2
  // q ≈ (0, sin(π/4), 0, cos(π/4)) gives ~90° pitch
  auto s = std::sin(M_PI / 4.0);
  auto c = std::cos(M_PI / 4.0);
  float roll = 0.0F;
  float pitch = 0.0F;
  float yaw = 0.0F;
  quaternion_to_euler(0.0, s, 0.0, c, roll, pitch, yaw);
  EXPECT_LE(std::abs(pitch), static_cast<float>(M_PI / 2.0) + 1e-5F);
}

// --- encode_vision_position_estimate ---

TEST(EncodeVisionPositionEstimate, MsgIdIs102) {
  auto pose = make_pose(0.0F, 0.0F, 0.0F, 0.0, 0.0, 0.0, 1.0);
  auto msg = encode_vision_position_estimate(pose, 0, 1, 195);
  EXPECT_EQ(msg.msgid, kMsgIdVisionPositionEstimate);
}

TEST(EncodeVisionPositionEstimate, PayloadHasFourElements) {
  auto pose = make_pose(0.0F, 0.0F, 0.0F, 0.0, 0.0, 0.0, 1.0);
  auto msg = encode_vision_position_estimate(pose, 0, 1, 195);
  EXPECT_EQ(msg.payload64.size(), 4u);
}

TEST(EncodeVisionPositionEstimate, FramingStatusOk) {
  auto pose = make_pose(0.0F, 0.0F, 0.0F, 0.0, 0.0, 0.0, 1.0);
  auto msg = encode_vision_position_estimate(pose, 0, 1, 195);
  EXPECT_EQ(msg.framing_status, mavros_msgs::msg::Mavlink::FRAMING_OK);
}

TEST(EncodeVisionPositionEstimate, Mavlink1Magic) {
  auto pose = make_pose(0.0F, 0.0F, 0.0F, 0.0, 0.0, 0.0, 1.0);
  auto msg = encode_vision_position_estimate(pose, 0, 1, 195);
  EXPECT_EQ(msg.magic, kMavlink1Magic);
}

TEST(EncodeVisionPositionEstimate, PayloadLen32) {
  auto pose = make_pose(0.0F, 0.0F, 0.0F, 0.0, 0.0, 0.0, 1.0);
  auto msg = encode_vision_position_estimate(pose, 0, 1, 195);
  EXPECT_EQ(msg.len, kVisionPosPayloadLen);
}

TEST(EncodeVisionPositionEstimate, SeqSysidCompidSet) {
  auto pose = make_pose(0.0F, 0.0F, 0.0F, 0.0, 0.0, 0.0, 1.0);
  auto msg = encode_vision_position_estimate(pose, 7, 42, 195);
  EXPECT_EQ(msg.seq, 7u);
  EXPECT_EQ(msg.sysid, 42u);
  EXPECT_EQ(msg.compid, 195u);
}

TEST(EncodeVisionPositionEstimate, TimestampFromHeader) {
  // sec=2, nanosec=500000000 → usec = 2*1e6 + 500000 = 2500000
  auto pose = make_pose(0.0F, 0.0F, 0.0F, 0.0, 0.0, 0.0, 1.0, 2, 500000000);
  auto msg = encode_vision_position_estimate(pose, 0, 1, 195);
  auto p = decode_payload(msg);
  EXPECT_EQ(p.usec, 2500000u);
}

TEST(EncodeVisionPositionEstimate, PositionEncodedCorrectly) {
  auto pose = make_pose(1.5F, 2.5F, 3.5F, 0.0, 0.0, 0.0, 1.0);
  auto msg = encode_vision_position_estimate(pose, 0, 1, 195);
  auto p = decode_payload(msg);
  EXPECT_NEAR(p.x, 1.5F, 1e-6F);
  EXPECT_NEAR(p.y, 2.5F, 1e-6F);
  EXPECT_NEAR(p.z, 3.5F, 1e-6F);
}

TEST(EncodeVisionPositionEstimate, IdentityOrientationGivesZeroAngles) {
  auto pose = make_pose(0.0F, 0.0F, 0.0F, 0.0, 0.0, 0.0, 1.0);
  auto msg = encode_vision_position_estimate(pose, 0, 1, 195);
  auto p = decode_payload(msg);
  EXPECT_NEAR(p.roll, 0.0F, 1e-5F);
  EXPECT_NEAR(p.pitch, 0.0F, 1e-5F);
  EXPECT_NEAR(p.yaw, 0.0F, 1e-5F);
}

TEST(EncodeVisionPositionEstimate, Yaw90DegreesEncoded) {
  // 90° yaw quaternion
  auto s = static_cast<float>(std::sin(M_PI / 4.0));
  auto c = static_cast<float>(std::cos(M_PI / 4.0));
  auto pose = make_pose(0.0F, 0.0F, 0.0F, 0.0, 0.0, s, c);
  auto msg = encode_vision_position_estimate(pose, 0, 1, 195);
  auto p = decode_payload(msg);
  EXPECT_NEAR(p.yaw, static_cast<float>(M_PI / 2.0), 1e-5F);
}

TEST(EncodeVisionPositionEstimate, HeaderCopiedToMsg) {
  auto pose = make_pose(0.0F, 0.0F, 0.0F, 0.0, 0.0, 0.0, 1.0, 99, 0);
  auto msg = encode_vision_position_estimate(pose, 0, 1, 195);
  EXPECT_EQ(msg.header.stamp.sec, 99);
  EXPECT_EQ(msg.header.frame_id, "map");
}

TEST(EncodeVisionPositionEstimate, IncompatFlagsZero) {
  auto pose = make_pose(0.0F, 0.0F, 0.0F, 0.0, 0.0, 0.0, 1.0);
  auto msg = encode_vision_position_estimate(pose, 0, 1, 195);
  EXPECT_EQ(msg.incompat_flags, 0u);
  EXPECT_EQ(msg.compat_flags, 0u);
}

}  // namespace
}  // namespace pose_estimator
