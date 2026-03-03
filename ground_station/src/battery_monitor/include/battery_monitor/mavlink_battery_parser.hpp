#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace battery_monitor {

constexpr uint32_t kMsgIdSysStatus = 1;

constexpr uint8_t kMavlinkV1Stx = 0xFE;
constexpr uint8_t kMavlinkV2Stx = 0xFD;

constexpr size_t kMavlinkV1HeaderLen = 6;
constexpr size_t kMavlinkV2HeaderLen = 10;

// MAVLink SYS_STATUS (msgid=1) — battery fields only.
struct SysStatus {
  uint16_t voltage_battery;  // mV; UINT16_MAX = not available
  int16_t current_battery;   // cA (10*mA); -1 = not available
  int8_t battery_remaining;  // %; -1 = not available
};

/// Extract MAVLink message ID from a complete v1/v2 frame.
/// Returns 0 if the frame is too short or the STX byte is unrecognized.
uint32_t get_msg_id(const std::vector<uint8_t>& frame);

/// Parse SYS_STATUS battery fields from a complete MAVLink frame.
/// Returns false if the frame is not SYS_STATUS or is too short.
bool parse_sys_status(const std::vector<uint8_t>& frame, SysStatus& out);

}  // namespace battery_monitor
