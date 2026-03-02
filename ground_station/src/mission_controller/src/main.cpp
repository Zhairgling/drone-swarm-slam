#include <rclcpp/rclcpp.hpp>

#include "mission_controller/mission_controller_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mission_controller::MissionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
