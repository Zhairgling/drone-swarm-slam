#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "pointcloud_assembler/pointcloud_assembler.hpp"

namespace pointcloud_assembler {

/// ROS 2 node that subscribes to per-drone ToF PointCloud2 topics and
/// publishes an assembled cloud on /slam/map_3d.
///
/// Parameters:
///   drone_ids         (int[])  — drone IDs to subscribe to (default: [1])
///   window_duration_sec (double) — time window for accumulation (default: 1.0)
///   max_clouds        (int)   — max clouds in buffer (default: 100)
///   publish_rate_hz   (double) — assembled cloud publish rate (default: 10.0)
///   output_frame      (string) — target TF frame for assembled cloud
///                                (default: "map")
class PointcloudAssemblerNode : public rclcpp::Node {
 public:
  explicit PointcloudAssemblerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void on_pointcloud(sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publish_timer_callback();

  std::unique_ptr<PointcloudAssembler> assembler_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      subscriptions_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::string output_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace pointcloud_assembler
