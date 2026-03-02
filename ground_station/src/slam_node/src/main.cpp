#include <rclcpp/rclcpp.hpp>

#include "slam_node/slam_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam_node::SlamNode>());
  rclcpp::shutdown();
  return 0;
}
