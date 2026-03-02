#include <rclcpp/rclcpp.hpp>

#include "pointcloud_assembler/pointcloud_assembler_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<pointcloud_assembler::PointcloudAssemblerNode>());
  rclcpp::shutdown();
  return 0;
}
