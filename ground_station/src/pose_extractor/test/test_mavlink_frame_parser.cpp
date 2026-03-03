// cppcheck-suppress-file syntaxError
#include "pose_extractor/mavlink_frame_parser.hpp"

#include <cmath>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mavlink_pose_extractor {
namespace {

// Build a minimal MAVLink v1 frame for a given msgid and payload.
std::vector<uint8_t> make_v1_frame(uint8_t msgid,
                                   const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> frame;
  frame.push_back(kMavlinkV1Stx);
  frame.push_back(static_cast<uint8_t>(payload.size()));
  frame.push_back(0);   // seq
  frame.push_back(1);   // sysid
  frame.push_back(1);   // compid
  frame.push_back(msgid);
  frame.insert(frame.end(), payload.begin(), payload.end());
  frame.push_back(0x00);  // CRC low (not validated)
  frame.push_back(0x00);  // CRC high
  return frame;
}

// Build a minimal MAVLink v2 frame for a given msgid and payload.
std::vector<uint8_t> make_v2_frame(uint32_t msgid,
                                   const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> frame;
  frame.push_back(kMavlinkV2Stx);
  frame.push_back(static_cast<uint8_t>(payload.size()));
  frame.push_back(0);  // incompat_flags
  frame.push_back(0);  // compat_flags
  frame.push_back(0);  // seq
  frame.push_back(1);  // sysid
  frame.push_back(1);  // compid
  frame.push_back(static_cast<uint8_t>(msgid & 0xFFu));
  frame.push_back(static_cast<uint8_t>((msgid >> 8u) & 0xFFu));
  frame.push_back(static_cast<uint8_t>((msgid >> 16u) & 0xFFu));
  frame.insert(frame.end(), payload.begin(), payload.end());
  frame.push_back(0x00);  // CRC low
  frame.push_back(0x00);  // CRC high
  return frame;
}

// Build a 28-byte LOCAL_POSITION_NED payload.
std::vector<uint8_t> make_pos_payload(uint32_t time_ms, float x, float y,
                                      float z) {
  std::vector<uint8_t> p(28, 0);
  std::memcpy(p.data(), &time_ms, 4);
  std::memcpy(p.data() + 4, &x, 4);
  std::memcpy(p.data() + 8, &y, 4);
  std::memcpy(p.data() + 12, &z, 4);
  return p;
}

// Build a 28-byte ATTITUDE payload.
std::vector<uint8_t> make_att_payload(uint32_t time_ms, float roll,
                                      float pitch, float yaw) {
  std::vector<uint8_t> p(28, 0);
  std::memcpy(p.data(), &time_ms, 4);
  std::memcpy(p.data() + 4, &roll, 4);
  std::memcpy(p.data() + 8, &pitch, 4);
  std::memcpy(p.data() + 12, &yaw, 4);
  return p;
}

// --- get_msg_id ---

TEST(GetMsgId, EmptyFrameReturnsZero) {
  EXPECT_EQ(get_msg_id({}), 0u);
}

TEST(GetMsgId, V1MsgIdExtracted) {
  auto frame =
      make_v1_frame(32, make_pos_payload(0, 0.0F, 0.0F, 0.0F));
  EXPECT_EQ(get_msg_id(frame), 32u);
}

TEST(GetMsgId, V1AttitudeMsgId) {
  auto frame = make_v1_frame(30, make_att_payload(0, 0.0F, 0.0F, 0.0F));
  EXPECT_EQ(get_msg_id(frame), 30u);
}

TEST(GetMsgId, V2MsgIdExtracted) {
  auto frame =
      make_v2_frame(32, make_pos_payload(0, 0.0F, 0.0F, 0.0F));
  EXPECT_EQ(get_msg_id(frame), 32u);
}

TEST(GetMsgId, V2LargeMsgId) {
  // Use msgid = 0x010203 (3-byte)
  std::vector<uint8_t> dummy(4, 0);
  auto frame = make_v2_frame(0x010203u, dummy);
  EXPECT_EQ(get_msg_id(frame), 0x010203u);
}

TEST(GetMsgId, UnknownStxReturnsZero) {
  std::vector<uint8_t> frame = {0xAB, 0x04, 0x00, 0x01, 0x01, 0x20};
  EXPECT_EQ(get_msg_id(frame), 0u);
}

TEST(GetMsgId, V1TooShortReturnsZero) {
  // Only 5 bytes — need at least 6 for v1 header
  std::vector<uint8_t> frame = {kMavlinkV1Stx, 0x04, 0x00, 0x01, 0x01};
  EXPECT_EQ(get_msg_id(frame), 0u);
}

// --- parse_local_position_ned ---

TEST(ParseLocalPositionNed, V1CorrectFrame) {
  auto payload = make_pos_payload(1000, 1.5F, 2.5F, -3.0F);
  auto frame = make_v1_frame(32, payload);

  LocalPositionNed out{};
  EXPECT_TRUE(parse_local_position_ned(frame, out));
  EXPECT_EQ(out.time_boot_ms, 1000u);
  EXPECT_NEAR(out.x, 1.5F, 1e-6F);
  EXPECT_NEAR(out.y, 2.5F, 1e-6F);
  EXPECT_NEAR(out.z, -3.0F, 1e-6F);
}

TEST(ParseLocalPositionNed, V2CorrectFrame) {
  auto payload = make_pos_payload(500, 0.1F, 0.2F, 0.3F);
  auto frame = make_v2_frame(32, payload);

  LocalPositionNed out{};
  EXPECT_TRUE(parse_local_position_ned(frame, out));
  EXPECT_NEAR(out.x, 0.1F, 1e-6F);
  EXPECT_NEAR(out.y, 0.2F, 1e-6F);
  EXPECT_NEAR(out.z, 0.3F, 1e-6F);
}

TEST(ParseLocalPositionNed, WrongMsgIdReturnsFalse) {
  auto payload = make_pos_payload(0, 0.0F, 0.0F, 0.0F);
  auto frame = make_v1_frame(30, payload);  // ATTITUDE, not LOCAL_POSITION_NED

  LocalPositionNed out{};
  EXPECT_FALSE(parse_local_position_ned(frame, out));
}

TEST(ParseLocalPositionNed, TooShortReturnsFalse) {
  // Frame with only 10-byte payload — less than required 16
  std::vector<uint8_t> payload(10, 0);
  auto frame = make_v1_frame(32, payload);

  LocalPositionNed out{};
  EXPECT_FALSE(parse_local_position_ned(frame, out));
}

// --- parse_attitude ---

TEST(ParseAttitude, V1CorrectFrame) {
  auto payload = make_att_payload(2000, 0.1F, 0.2F, 1.57F);
  auto frame = make_v1_frame(30, payload);

  Attitude out{};
  EXPECT_TRUE(parse_attitude(frame, out));
  EXPECT_EQ(out.time_boot_ms, 2000u);
  EXPECT_NEAR(out.roll, 0.1F, 1e-6F);
  EXPECT_NEAR(out.pitch, 0.2F, 1e-6F);
  EXPECT_NEAR(out.yaw, 1.57F, 1e-5F);
}

TEST(ParseAttitude, V2CorrectFrame) {
  auto payload = make_att_payload(0, 0.0F, 0.0F, 0.0F);
  auto frame = make_v2_frame(30, payload);

  Attitude out{};
  EXPECT_TRUE(parse_attitude(frame, out));
  EXPECT_NEAR(out.roll, 0.0F, 1e-6F);
  EXPECT_NEAR(out.pitch, 0.0F, 1e-6F);
  EXPECT_NEAR(out.yaw, 0.0F, 1e-6F);
}

TEST(ParseAttitude, WrongMsgIdReturnsFalse) {
  auto payload = make_att_payload(0, 0.0F, 0.0F, 0.0F);
  auto frame = make_v1_frame(32, payload);  // LOCAL_POSITION_NED, not ATTITUDE

  Attitude out{};
  EXPECT_FALSE(parse_attitude(frame, out));
}

TEST(ParseAttitude, TooShortReturnsFalse) {
  std::vector<uint8_t> payload(10, 0);
  auto frame = make_v1_frame(30, payload);

  Attitude out{};
  EXPECT_FALSE(parse_attitude(frame, out));
}

// --- ned_attitude_to_enu_quaternion ---

// Floating-point tolerance for quaternion comparisons.
// The NED→ENU conversion involves several trig operations; 1e-6 is sufficient.
constexpr double kQuatTol = 1e-6;

// Verify quaternion is unit length
static void expect_unit_quaternion(double qx, double qy, double qz,
                                   double qw) {
  double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  EXPECT_NEAR(norm, 1.0, kQuatTol);
}

// Identity in NED (facing North) → 90° CCW yaw in ENU (still facing North)
// ENU: yaw=90° → q = (0, 0, sin(45°), cos(45°)) = (0, 0, 1/√2, 1/√2)
TEST(NedToEnuQuaternion, IdentityNedFacesNorthInEnu) {
  double qx{}, qy{}, qz{}, qw{};
  ned_attitude_to_enu_quaternion(0.0F, 0.0F, 0.0F, qx, qy, qz, qw);

  expect_unit_quaternion(qx, qy, qz, qw);
  EXPECT_GE(qw, 0.0);  // canonical form
  // Should be a 90° yaw in ENU (facing North)
  const double expected_z = 1.0 / std::sqrt(2.0);
  const double expected_w = 1.0 / std::sqrt(2.0);
  EXPECT_NEAR(std::abs(qx), 0.0, kQuatTol);
  EXPECT_NEAR(std::abs(qy), 0.0, kQuatTol);
  EXPECT_NEAR(std::abs(qz), expected_z, kQuatTol);
  EXPECT_NEAR(std::abs(qw), expected_w, kQuatTol);
}

// yaw_ned=90° (facing East) → identity in ENU (East = ENU x-axis)
TEST(NedToEnuQuaternion, Yaw90NedFacesEastInEnu) {
  double qx{}, qy{}, qz{}, qw{};
  ned_attitude_to_enu_quaternion(0.0F, 0.0F,
                                 static_cast<float>(M_PI / 2.0), qx, qy, qz,
                                 qw);

  expect_unit_quaternion(qx, qy, qz, qw);
  EXPECT_GE(qw, 0.0);
  // Identity in ENU: (0, 0, 0, 1)
  EXPECT_NEAR(qx, 0.0, kQuatTol);
  EXPECT_NEAR(qy, 0.0, kQuatTol);
  EXPECT_NEAR(qz, 0.0, kQuatTol);
  EXPECT_NEAR(qw, 1.0, kQuatTol);
}

// yaw_ned=π (facing South) → -90° yaw in ENU
TEST(NedToEnuQuaternion, Yaw180NedFacesSouthInEnu) {
  double qx{}, qy{}, qz{}, qw{};
  ned_attitude_to_enu_quaternion(0.0F, 0.0F, static_cast<float>(M_PI), qx, qy,
                                 qz, qw);

  expect_unit_quaternion(qx, qy, qz, qw);
  EXPECT_GE(qw, 0.0);
  // -90° yaw in ENU → q = (0, 0, -1/√2, 1/√2), canonical: (0,0,-1/√2,1/√2)
  const double expected = 1.0 / std::sqrt(2.0);
  EXPECT_NEAR(std::abs(qx), 0.0, kQuatTol);
  EXPECT_NEAR(std::abs(qy), 0.0, kQuatTol);
  EXPECT_NEAR(std::abs(qz), expected, kQuatTol);
  EXPECT_NEAR(std::abs(qw), expected, kQuatTol);
}

TEST(NedToEnuQuaternion, OutputIsAlwaysUnitQuaternion) {
  // Check several arbitrary orientations
  const float angles[][3] = {
      {0.1F, 0.2F, 0.3F},
      {-0.5F, 0.4F, 1.2F},
      {0.0F, 0.0F, static_cast<float>(-M_PI / 3.0)},
      {static_cast<float>(M_PI / 4.0), 0.0F, 0.0F},
  };
  for (const auto& a : angles) {
    double qx{}, qy{}, qz{}, qw{};
    ned_attitude_to_enu_quaternion(a[0], a[1], a[2], qx, qy, qz, qw);
    expect_unit_quaternion(qx, qy, qz, qw);
    EXPECT_GE(qw, 0.0);
  }
}

TEST(NedToEnuQuaternion, CanonicalFormHasNonNegativeW) {
  double qx{}, qy{}, qz{}, qw{};
  // This case previously produced negative qw before canonical normalization
  ned_attitude_to_enu_quaternion(0.0F, 0.0F, 0.0F, qx, qy, qz, qw);
  EXPECT_GE(qw, 0.0);
}

}  // namespace
}  // namespace mavlink_pose_extractor
