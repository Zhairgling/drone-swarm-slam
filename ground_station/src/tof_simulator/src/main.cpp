#include <rclcpp/rclcpp.hpp>

#include "tof_simulator/tof_simulator_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tof_simulator::TofSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
