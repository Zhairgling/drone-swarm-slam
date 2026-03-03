#include "pose_extractor/mavlink_frame_parser.hpp"

#include <cmath>
#include <cstring>

namespace mavlink_pose_extractor {

namespace {

// Returns the payload start offset for a valid frame, or 0 for unknown STX.
size_t payload_offset(uint8_t stx) {
  if (stx == kMavlinkV1Stx) return kMavlinkV1HeaderLen;
  if (stx == kMavlinkV2Stx) return kMavlinkV2HeaderLen;
  return 0;
}

// Read a little-endian uint32_t from raw bytes without strict-aliasing UB.
uint32_t read_u32_le(const uint8_t* p) {
  uint32_t v;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

// Read a little-endian float from raw bytes without strict-aliasing UB.
float read_f32_le(const uint8_t* p) {
  float v;
  std::memcpy(&v, p, sizeof(v));
  return v;
}

}  // namespace

uint32_t get_msg_id(const std::vector<uint8_t>& frame) {
  if (frame.empty()) return 0;

  if (frame[0] == kMavlinkV1Stx) {
    // v1: msgid is a single byte at offset 5
    if (frame.size() < kMavlinkV1HeaderLen) return 0;
    return frame[5];
  }

  if (frame[0] == kMavlinkV2Stx) {
    // v2: msgid is 3 bytes (little-endian) at offsets 7, 8, 9
    if (frame.size() < kMavlinkV2HeaderLen) return 0;
    return static_cast<uint32_t>(frame[7]) |
           (static_cast<uint32_t>(frame[8]) << 8u) |
           (static_cast<uint32_t>(frame[9]) << 16u);
  }

  return 0;
}

bool parse_local_position_ned(const std::vector<uint8_t>& frame,
                               LocalPositionNed& out) {
  if (get_msg_id(frame) != kMsgIdLocalPositionNed) return false;

  size_t offset = payload_offset(frame[0]);
  // Payload: time_boot_ms(4) + x(4) + y(4) + z(4) + vx/vy/vz(12) = 28 bytes
  constexpr size_t kRequiredPayload = 16;  // we only read the first 16 bytes
  if (frame.size() < offset + kRequiredPayload) return false;

  const uint8_t* p = frame.data() + offset;
  out.time_boot_ms = read_u32_le(p);
  out.x            = read_f32_le(p + 4);
  out.y            = read_f32_le(p + 8);
  out.z            = read_f32_le(p + 12);
  return true;
}

bool parse_attitude(const std::vector<uint8_t>& frame, Attitude& out) {
  if (get_msg_id(frame) != kMsgIdAttitude) return false;

  size_t offset = payload_offset(frame[0]);
  // Payload: time_boot_ms(4) + roll(4) + pitch(4) + yaw(4) + rates(12) = 28
  constexpr size_t kRequiredPayload = 16;  // we only read the first 16 bytes
  if (frame.size() < offset + kRequiredPayload) return false;

  const uint8_t* p = frame.data() + offset;
  out.time_boot_ms = read_u32_le(p);
  out.roll         = read_f32_le(p + 4);
  out.pitch        = read_f32_le(p + 8);
  out.yaw          = read_f32_le(p + 12);
  return true;
}

void ned_attitude_to_enu_quaternion(float roll_ned, float pitch_ned,
                                    float yaw_ned, double& qx, double& qy,
                                    double& qz, double& qw) {
  // Build ZYX (aerospace) quaternion from NED roll/pitch/yaw.
  // q_ned = q_z(yaw) * q_y(pitch) * q_x(roll)
  const double cr = std::cos(roll_ned * 0.5);
  const double sr = std::sin(roll_ned * 0.5);
  const double cp = std::cos(pitch_ned * 0.5);
  const double sp = std::sin(pitch_ned * 0.5);
  const double cy = std::cos(yaw_ned * 0.5);
  const double sy = std::sin(yaw_ned * 0.5);

  const double qw_n = cr * cp * cy + sr * sp * sy;
  const double qx_n = sr * cp * cy - cr * sp * sy;
  const double qy_n = cr * sp * cy + sr * cp * sy;
  const double qz_n = cr * cp * sy - sr * sp * cy;

  // DESIGN: Full NED/FRD → ENU/FLU conversion:
  //   q_enu = q_ned2enu * q_ned * q_frd2flu
  // where q_ned2enu = (1/√2, 1/√2, 0, 0)  [180° about NE axis]
  //   and q_frd2flu = (1, 0, 0, 0)          [180° about x-axis]
  //
  // Closed-form after expanding the two quaternion products:
  const double s = 1.0 / std::sqrt(2.0);
  qx = -s * (qx_n + qy_n);
  qy =  s * (qy_n - qx_n);
  qz =  s * (qz_n - qw_n);
  qw = -s * (qw_n + qz_n);

  // Normalize for floating-point safety.
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  qx /= norm;
  qy /= norm;
  qz /= norm;
  qw /= norm;

  // Canonical form: ensure qw >= 0 (q and -q represent the same rotation).
  if (qw < 0.0) {
    qx = -qx;
    qy = -qy;
    qz = -qz;
    qw = -qw;
  }
}

}  // namespace mavlink_pose_extractor
