// cppcheck-suppress-file syntaxError
#include "battery_monitor/mavlink_battery_parser.hpp"

#include <cstring>
#include <vector>

#include <gtest/gtest.h>

namespace battery_monitor {
namespace {

// Build a minimal MAVLink v1 frame.
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

// Build a minimal MAVLink v2 frame.
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

// Build a 31-byte SYS_STATUS payload with given battery values.
std::vector<uint8_t> make_sys_status_payload(uint16_t voltage_mv,
                                             int16_t current_ca,
                                             int8_t battery_pct) {
  std::vector<uint8_t> p(31, 0);
  // voltage_battery at offset 14, current_battery at 16, battery_remaining at 30
  std::memcpy(p.data() + 14, &voltage_mv, sizeof(voltage_mv));
  std::memcpy(p.data() + 16, &current_ca, sizeof(current_ca));
  p[30] = static_cast<uint8_t>(battery_pct);
  return p;
}

// --- get_msg_id ---

TEST(GetMsgId, EmptyFrameReturnsZero) {
  EXPECT_EQ(get_msg_id({}), 0u);
}

TEST(GetMsgId, V1SysStatusMsgId) {
  auto payload = make_sys_status_payload(14800, 100, 80);
  auto frame = make_v1_frame(1, payload);
  EXPECT_EQ(get_msg_id(frame), 1u);
}

TEST(GetMsgId, V2SysStatusMsgId) {
  auto payload = make_sys_status_payload(14800, 100, 80);
  auto frame = make_v2_frame(1, payload);
  EXPECT_EQ(get_msg_id(frame), 1u);
}

TEST(GetMsgId, UnknownStxReturnsZero) {
  std::vector<uint8_t> frame = {0xAB, 0x04, 0x00, 0x01, 0x01, 0x01};
  EXPECT_EQ(get_msg_id(frame), 0u);
}

TEST(GetMsgId, V1TooShortReturnsZero) {
  std::vector<uint8_t> frame = {kMavlinkV1Stx, 0x1F, 0x00, 0x01, 0x01};
  EXPECT_EQ(get_msg_id(frame), 0u);
}

// --- parse_sys_status ---

TEST(ParseSysStatus, V1CorrectFrame) {
  // 14800 mV = 14.8 V, 250 cA = 2.5 A, 85%
  auto payload = make_sys_status_payload(14800, 250, 85);
  auto frame = make_v1_frame(1, payload);

  SysStatus out{};
  EXPECT_TRUE(parse_sys_status(frame, out));
  EXPECT_EQ(out.voltage_battery, 14800u);
  EXPECT_EQ(out.current_battery, 250);
  EXPECT_EQ(out.battery_remaining, 85);
}

TEST(ParseSysStatus, V2CorrectFrame) {
  auto payload = make_sys_status_payload(13600, 300, 10);
  auto frame = make_v2_frame(1, payload);

  SysStatus out{};
  EXPECT_TRUE(parse_sys_status(frame, out));
  EXPECT_EQ(out.voltage_battery, 13600u);
  EXPECT_EQ(out.current_battery, 300);
  EXPECT_EQ(out.battery_remaining, 10);
}

TEST(ParseSysStatus, WrongMsgIdReturnsFalse) {
  // msgid=2 (SYS_STATUS is 1)
  auto payload = make_sys_status_payload(14800, 100, 80);
  auto frame = make_v1_frame(2, payload);

  SysStatus out{};
  EXPECT_FALSE(parse_sys_status(frame, out));
}

TEST(ParseSysStatus, TooShortReturnsFalse) {
  // Only 20 bytes payload — need 31
  std::vector<uint8_t> payload(20, 0);
  auto frame = make_v1_frame(1, payload);

  SysStatus out{};
  EXPECT_FALSE(parse_sys_status(frame, out));
}

TEST(ParseSysStatus, NotAvailableValues) {
  // voltage = UINT16_MAX (not available), current = -1, remaining = -1
  auto payload = make_sys_status_payload(0xFFFF, -1, -1);
  auto frame = make_v1_frame(1, payload);

  SysStatus out{};
  EXPECT_TRUE(parse_sys_status(frame, out));
  EXPECT_EQ(out.voltage_battery, 0xFFFFu);
  EXPECT_EQ(out.current_battery, -1);
  EXPECT_EQ(out.battery_remaining, -1);
}

TEST(ParseSysStatus, MinimumVoltage) {
  // 13200 mV = 13.2V (critically low 4S)
  auto payload = make_sys_status_payload(13200, 50, 5);
  auto frame = make_v1_frame(1, payload);

  SysStatus out{};
  EXPECT_TRUE(parse_sys_status(frame, out));
  EXPECT_EQ(out.voltage_battery, 13200u);
  EXPECT_EQ(out.battery_remaining, 5);
}

}  // namespace
}  // namespace battery_monitor
