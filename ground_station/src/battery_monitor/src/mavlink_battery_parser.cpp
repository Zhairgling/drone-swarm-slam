#include "battery_monitor/mavlink_battery_parser.hpp"

#include <cstring>

namespace battery_monitor {

namespace {

// Returns the payload start offset for a valid frame, or 0 for unknown STX.
size_t payload_offset(uint8_t stx) {
  if (stx == kMavlinkV1Stx) return kMavlinkV1HeaderLen;
  if (stx == kMavlinkV2Stx) return kMavlinkV2HeaderLen;
  return 0;
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

bool parse_sys_status(const std::vector<uint8_t>& frame, SysStatus& out) {
  if (get_msg_id(frame) != kMsgIdSysStatus) return false;

  size_t offset = payload_offset(frame[0]);
  // DESIGN: SYS_STATUS wire layout (MAVLink field-reordering by type size):
  //   uint32 onboard_control_sensors_present  [0-3]
  //   uint32 onboard_control_sensors_enabled  [4-7]
  //   uint32 onboard_control_sensors_health   [8-11]
  //   uint16 load                             [12-13]
  //   uint16 voltage_battery                  [14-15]  <- we read this
  //   int16  current_battery                  [16-17]  <- we read this
  //   uint16 drop_rate_comm                   [18-19]
  //   uint16 errors_comm                      [20-21]
  //   uint16 errors_count1                    [22-23]
  //   uint16 errors_count2                    [24-25]
  //   uint16 errors_count3                    [26-27]
  //   uint16 errors_count4                    [28-29]
  //   int8   battery_remaining                [30]     <- we read this
  constexpr size_t kRequiredPayload = 31;
  if (frame.size() < offset + kRequiredPayload) return false;

  const uint8_t* p = frame.data() + offset;
  std::memcpy(&out.voltage_battery, p + 14, sizeof(out.voltage_battery));
  std::memcpy(&out.current_battery, p + 16, sizeof(out.current_battery));
  out.battery_remaining = static_cast<int8_t>(p[30]);
  return true;
}

}  // namespace battery_monitor
