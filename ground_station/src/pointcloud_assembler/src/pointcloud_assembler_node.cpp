#include "pointcloud_assembler/pointcloud_assembler_node.hpp"

#include <chrono>
#include <functional>

namespace pointcloud_assembler {

PointcloudAssemblerNode::PointcloudAssemblerNode(
    const rclcpp::NodeOptions& options)
    : Node("pointcloud_assembler", options) {
  declare_parameter("drone_ids", std::vector<int64_t>{1});
  declare_parameter("window_duration_sec", 1.0);
  declare_parameter("max_clouds", 100);
  declare_parameter("publish_rate_hz", 10.0);
  declare_parameter("output_frame", std::string("odom"));

  AssemblerConfig config;
  config.window_duration_sec =
      get_parameter("window_duration_sec").as_double();
  config.max_clouds =
      static_cast<size_t>(get_parameter("max_clouds").as_int());
  assembler_ = std::make_unique<PointcloudAssembler>(config);

  output_frame_ = get_parameter("output_frame").as_string();

  // DESIGN: best_effort QoS matches real micro-ROS sensor publishers
  // (see CLAUDE.md QoS section).
  auto qos_in = rclcpp::QoS(rclcpp::KeepLast(10))
                    .reliability(rclcpp::ReliabilityPolicy::BestEffort);

  auto drone_ids = get_parameter("drone_ids").as_integer_array();
  for (const auto id : drone_ids) {
    std::string topic =
        "/drone_" + std::to_string(id) + "/tof/pointcloud";
    auto sub = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, qos_in,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          on_pointcloud(std::move(msg));
        });
    subscriptions_.push_back(sub);
    RCLCPP_INFO(get_logger(), "Subscribing to %s", topic.c_str());
  }

  // DESIGN: reliable QoS for the assembled output — downstream SLAM consumers
  // cannot afford to miss assembled frames.
  auto qos_out = rclcpp::QoS(rclcpp::KeepLast(5))
                     .reliability(rclcpp::ReliabilityPolicy::Reliable);
  publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/slam/map_3d", qos_out);

  double rate_hz = get_parameter("publish_rate_hz").as_double();
  auto period = std::chrono::duration<double>(1.0 / rate_hz);
  publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PointcloudAssemblerNode::publish_timer_callback, this));

  RCLCPP_INFO(get_logger(),
              "PointcloudAssembler started: %zu drone(s), "
              "window=%.1fs, rate=%.1fHz, output_frame=%s",
              drone_ids.size(), config.window_duration_sec, rate_hz,
              output_frame_.c_str());
}

void PointcloudAssemblerNode::on_pointcloud(
    sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  assembler_->add_cloud(*msg);
}

void PointcloudAssemblerNode::publish_timer_callback() {
  auto now = this->get_clock()->now();
  int64_t now_ns = now.nanoseconds();

  assembler_->expire_old_clouds(now_ns);

  auto sec = static_cast<int32_t>(now_ns / 1000000000LL);
  auto nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
  auto cloud = assembler_->assemble(output_frame_, sec, nanosec);

  publisher_->publish(cloud);
}

}  // namespace pointcloud_assembler
