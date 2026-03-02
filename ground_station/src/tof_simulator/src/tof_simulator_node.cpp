#include "tof_simulator/tof_simulator_node.hpp"

#include <chrono>

namespace tof_simulator {

TofSimulatorNode::TofSimulatorNode(const rclcpp::NodeOptions& options)
    : Node("tof_simulator", options) {
  // Declare parameters with defaults
  declare_parameter("drone_id", 1);
  declare_parameter("publish_rate_hz", 15.0);
  declare_parameter("fov_deg", kDefaultFovDeg);
  declare_parameter("max_range_mm", kDefaultMaxRangeMm);
  declare_parameter("noise_stddev_mm", kDefaultNoiseMm);
  declare_parameter("room_size_x", 5.0);
  declare_parameter("room_size_y", 5.0);
  declare_parameter("room_size_z", 3.0);
  declare_parameter("drone_pos_x", 2.5);
  declare_parameter("drone_pos_y", 2.5);
  declare_parameter("drone_pos_z", 1.5);

  // Build config from parameters
  SimulatorConfig config;
  config.drone_id = get_parameter("drone_id").as_int();
  config.fov_deg = get_parameter("fov_deg").as_double();
  config.max_range_mm = get_parameter("max_range_mm").as_double();
  config.noise_stddev_mm = get_parameter("noise_stddev_mm").as_double();
  config.room.size_x = get_parameter("room_size_x").as_double();
  config.room.size_y = get_parameter("room_size_y").as_double();
  config.room.size_z = get_parameter("room_size_z").as_double();
  config.drone_position.x = get_parameter("drone_pos_x").as_double();
  config.drone_position.y = get_parameter("drone_pos_y").as_double();
  config.drone_position.z = get_parameter("drone_pos_z").as_double();

  generator_ = std::make_unique<TofDataGenerator>(config);
  frame_id_ = "drone_" + std::to_string(config.drone_id) + "/base_link";

  // DESIGN: best_effort QoS matches the real micro-ROS sensor publishers
  // (see CLAUDE.md QoS section).
  auto qos = rclcpp::QoS(rclcpp::KeepLast(5))
                 .reliability(rclcpp::ReliabilityPolicy::BestEffort);

  std::string topic =
      "/drone_" + std::to_string(config.drone_id) + "/tof/pointcloud";
  publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic, qos);

  double rate_hz = get_parameter("publish_rate_hz").as_double();
  auto period = std::chrono::duration<double>(1.0 / rate_hz);
  timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TofSimulatorNode::timer_callback, this));

  RCLCPP_INFO(get_logger(),
              "ToF simulator started: drone_id=%d, topic=%s, rate=%.1f Hz, "
              "%d points/frame",
              config.drone_id, topic.c_str(), rate_hz,
              generator_->points_per_frame());
}

void TofSimulatorNode::timer_callback() {
  auto now = this->get_clock()->now();
  auto total_ns = now.nanoseconds();
  auto sec = static_cast<int32_t>(total_ns / 1000000000LL);
  auto nanosec = static_cast<uint32_t>(total_ns % 1000000000LL);
  auto cloud = generator_->generate_pointcloud(frame_id_, sec, nanosec);
  publisher_->publish(cloud);
}

}  // namespace tof_simulator
