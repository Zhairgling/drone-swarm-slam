#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

TEST(GroundStationTest, RclcppInit) {
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_node");
  EXPECT_NE(node, nullptr);
  rclcpp::shutdown();
}
