#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mavlink_pose_extractor {

constexpr uint32_t kMsgIdAttitude = 30;
constexpr uint32_t kMsgIdLocalPositionNed = 32;

constexpr uint8_t kMavlinkV1Stx = 0xFE;
constexpr uint8_t kMavlinkV2Stx = 0xFD;

constexpr size_t kMavlinkV1HeaderLen = 6;
constexpr size_t kMavlinkV2HeaderLen = 10;

// MAVLink LOCAL_POSITION_NED (msgid=32) payload — NED frame, meters.
struct LocalPositionNed {
  uint32_t time_boot_ms;  // cppcheck-suppress unusedStructMember
  float x;                // cppcheck-suppress unusedStructMember
  float y;                // cppcheck-suppress unusedStructMember
  float z;                // cppcheck-suppress unusedStructMember
};

// MAVLink ATTITUDE (msgid=30) payload — NED frame, radians.
struct Attitude {
  uint32_t time_boot_ms;  // cppcheck-suppress unusedStructMember
  float roll;             // cppcheck-suppress unusedStructMember
  float pitch;            // cppcheck-suppress unusedStructMember
  float yaw;              // cppcheck-suppress unusedStructMember
};

/// Extract MAVLink message ID from a complete v1/v2 frame.
/// Returns 0 if the frame is too short or the STX byte is unrecognized.
uint32_t get_msg_id(const std::vector<uint8_t>& frame);

/// Parse LOCAL_POSITION_NED payload from a complete MAVLink frame.
/// Returns false if the frame is not a LOCAL_POSITION_NED message or is
/// malformed.
bool parse_local_position_ned(const std::vector<uint8_t>& frame,
                               LocalPositionNed& out);

/// Parse ATTITUDE payload from a complete MAVLink frame.
/// Returns false if the frame is not an ATTITUDE message or is malformed.
bool parse_attitude(const std::vector<uint8_t>& frame, Attitude& out);

/// Convert NED roll/pitch/yaw (radians) to an ENU quaternion (xyzw).
///
/// ArduCopter uses NED world + FRD body frames; ROS uses ENU world + FLU body.
/// Full transformation: q_enu = q_ned2enu * q_ned * q_frd2flu
/// where q_ned2enu = (1/√2, 1/√2, 0, 0) and q_frd2flu = (1, 0, 0, 0).
///
/// Resulting quaternion has qw >= 0 (canonical form).
void ned_attitude_to_enu_quaternion(float roll_ned, float pitch_ned,
                                    float yaw_ned, double& qx, double& qy,
                                    double& qz, double& qw);

}  // namespace mavlink_pose_extractor
