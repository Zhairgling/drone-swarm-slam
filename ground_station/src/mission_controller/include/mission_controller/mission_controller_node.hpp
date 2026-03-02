#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mission_controller/waypoint_manager.hpp"

namespace mission_controller {

/// ROS 2 node that sequences waypoints for a single drone.
///
/// Subscribes to the drone's estimated pose and publishes the active waypoint
/// on /drone_N/mission/waypoint whenever the active waypoint changes or on a
/// periodic republish timer (so late-joining subscribers receive it).
class MissionControllerNode : public rclcpp::Node {
 public:
  explicit MissionControllerNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void pose_callback(
      const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
  void republish_timer_callback();
  void publish_current_waypoint();

  std::unique_ptr<WaypointManager> manager_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
};

}  // namespace mission_controller
