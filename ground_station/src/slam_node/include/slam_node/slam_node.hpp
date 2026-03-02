#pragma once

#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "slam_node/occupancy_map.hpp"

namespace slam_node {

/// ROS 2 SLAM node: accumulates ToF PointCloud2 into a 3D occupancy map.
///
/// Subscribes to /drone_N/tof/pointcloud for N = 1..num_drones.
/// Publishes /slam/map (nav_msgs/OccupancyGrid) at a configurable rate.
/// Publishes /slam/map_3d (sensor_msgs/PointCloud2) alongside the 2D map.
///
/// DESIGN: No TF2 transforms applied. Assumes incoming pointclouds are
/// expressed in a common map-relative frame. This matches the simulator
/// output and is sufficient until a pose estimator (issue #10) is added.
class SlamNode : public rclcpp::Node {
 public:
  explicit SlamNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publish_map();

  std::unique_ptr<OccupancyMap> map_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      subscriptions_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_3d_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::string map_frame_;
};

}  // namespace slam_node
