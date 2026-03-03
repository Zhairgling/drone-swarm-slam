#include "slam_node/slam_node.hpp"

#include <chrono>
#include <functional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace slam_node {

SlamNode::SlamNode(const rclcpp::NodeOptions& options)
    : Node("slam_node", options) {
  declare_parameter("num_drones", 1);
  declare_parameter("map_resolution", 0.1);
  declare_parameter("map_size_x", 20.0);
  declare_parameter("map_size_y", 20.0);
  declare_parameter("map_height_min", -0.5);
  declare_parameter("map_height_max", 3.0);
  declare_parameter("publish_rate_hz", 1.0);
  declare_parameter("map_frame", std::string("map"));

  map_frame_ = get_parameter("map_frame").as_string();

  MapConfig config;
  config.resolution = get_parameter("map_resolution").as_double();
  config.size_x = get_parameter("map_size_x").as_double();
  config.size_y = get_parameter("map_size_y").as_double();
  config.height_min = get_parameter("map_height_min").as_double();
  config.height_max = get_parameter("map_height_max").as_double();
  map_ = std::make_unique<OccupancyMap>(config);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // DESIGN: best_effort QoS matches the real micro-ROS sensor publishers
  // (see CLAUDE.md QoS section). Drone pointclouds are sensor data.
  auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5))
                        .reliability(rclcpp::ReliabilityPolicy::BestEffort);

  const int num_drones = get_parameter("num_drones").as_int();
  for (int i = 1; i <= num_drones; ++i) {
    const std::string topic =
        "/drone_" + std::to_string(i) + "/tof/pointcloud";
    auto sub = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, sensor_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          pointcloud_callback(msg);
        });
    subscriptions_.push_back(sub);
    RCLCPP_INFO(get_logger(), "Subscribed to: %s", topic.c_str());
  }

  // DESIGN: TransientLocal durability lets late-joining subscribers (e.g.,
  // RViz) receive the latest published map without waiting for the next tick.
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                     .reliability(rclcpp::ReliabilityPolicy::Reliable)
                     .durability(rclcpp::DurabilityPolicy::TransientLocal);
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/slam/map", map_qos);
  map_3d_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/slam/map_3d", map_qos);

  const double rate_hz = get_parameter("publish_rate_hz").as_double();
  const auto period = std::chrono::duration<double>(1.0 / rate_hz);
  publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SlamNode::publish_map, this));

  RCLCPP_INFO(get_logger(),
              "SLAM node started: %d drone(s), res=%.2fm, "
              "map=%.0fx%.0fm, publish=%.1fHz",
              num_drones, config.resolution, config.size_x, config.size_y,
              rate_hz);
}

void SlamNode::pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (msg->header.frame_id == map_frame_) {
    map_->add_pointcloud(*msg);
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    // DESIGN: 100ms timeout matches the 15Hz sensor rate (66ms period).
    // Clouds arriving before TF is available are skipped, not accumulated
    // in the wrong frame.
    transform = tf_buffer_->lookupTransform(
        map_frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_nanoseconds(100000000LL));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "TF not available for frame '%s': %s",
                         msg->header.frame_id.c_str(), ex.what());
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud_map;
  tf2::doTransform(*msg, cloud_map, transform);
  map_->add_pointcloud(cloud_map);
}

void SlamNode::publish_map() {
  const auto now = this->get_clock()->now();
  const int64_t total_ns = now.nanoseconds();
  const auto sec = static_cast<int32_t>(total_ns / 1000000000LL);
  const auto nanosec = static_cast<uint32_t>(total_ns % 1000000000LL);

  map_pub_->publish(map_->get_occupancy_grid(map_frame_, sec, nanosec));
  map_3d_pub_->publish(map_->get_point_cloud(map_frame_, sec, nanosec));
}

}  // namespace slam_node
