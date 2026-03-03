#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "pose_extractor/mavlink_pose_extractor_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<mavlink_pose_extractor::MavlinkPoseExtractorNode>());
  rclcpp::shutdown();
  return 0;
}
