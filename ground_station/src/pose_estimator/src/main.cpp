#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "pose_estimator/pose_estimator_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pose_estimator::PoseEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
