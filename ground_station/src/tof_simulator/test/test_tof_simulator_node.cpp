// cppcheck-suppress-file syntaxError
// Node-level tests for TofSimulatorNode.
//
// These tests cover node lifecycle and parameter handling.  Data quality is
// covered by test_tof_data_generator.cpp; publish/subscribe integration is
// better covered by launch tests where DDS discovery is fully initialised.

#include "tof_simulator/tof_simulator_node.hpp"

#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

namespace tof_simulator {
namespace {

class TofSimulatorNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

// ── Default parameters ────────────────────────────────────────────────────

TEST_F(TofSimulatorNodeTest, NodeCreatesWithDefaultParameters) {
  auto node = std::make_shared<TofSimulatorNode>();
  EXPECT_NE(node, nullptr);
  EXPECT_EQ(node->get_parameter("drone_id").as_int(), 1LL);
  EXPECT_DOUBLE_EQ(node->get_parameter("publish_rate_hz").as_double(), 15.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("noise_stddev_mm").as_double(), 20.0);
}

// ── Parameter overrides ───────────────────────────────────────────────────

TEST_F(TofSimulatorNodeTest, NodeAcceptsDroneIdOverride) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("drone_id", 5)});
  auto node = std::make_shared<TofSimulatorNode>(options);
  EXPECT_EQ(node->get_parameter("drone_id").as_int(), 5LL);
}

TEST_F(TofSimulatorNodeTest, NodeAcceptsPublishRateOverride) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("publish_rate_hz", 30.0)});
  auto node = std::make_shared<TofSimulatorNode>(options);
  EXPECT_DOUBLE_EQ(node->get_parameter("publish_rate_hz").as_double(), 30.0);
}

TEST_F(TofSimulatorNodeTest, NodeAcceptsRoomSizeOverrides) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("room_size_x", 10.0),
      rclcpp::Parameter("room_size_y", 8.0),
      rclcpp::Parameter("room_size_z", 4.0),
  });
  auto node = std::make_shared<TofSimulatorNode>(options);
  EXPECT_DOUBLE_EQ(node->get_parameter("room_size_x").as_double(), 10.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("room_size_y").as_double(), 8.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("room_size_z").as_double(), 4.0);
}

// ── Multiple drone IDs ────────────────────────────────────────────────────

TEST_F(TofSimulatorNodeTest, NodeCreatesForMultipleDroneIds) {
  // Verify the node initialises without error for several valid drone IDs.
  for (int id : {1, 2, 3, 10}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("drone_id", id)});
    EXPECT_NO_THROW({
      auto node = std::make_shared<TofSimulatorNode>(options);
      EXPECT_EQ(node->get_parameter("drone_id").as_int(),
                static_cast<int64_t>(id));
    }) << "Failed for drone_id=" << id;
  }
}

}  // namespace
}  // namespace tof_simulator

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
