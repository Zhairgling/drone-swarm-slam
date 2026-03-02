#include <rclcpp/rclcpp.hpp>

#include "tf_broadcaster/drone_tf_broadcaster.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tf_broadcaster::DroneTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
