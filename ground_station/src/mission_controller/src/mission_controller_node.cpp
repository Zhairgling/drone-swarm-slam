#include "mission_controller/mission_controller_node.hpp"

#include <chrono>
#include <string>
#include <vector>

namespace mission_controller {

MissionControllerNode::MissionControllerNode(const rclcpp::NodeOptions& options)
    : Node("mission_controller", options) {
  declare_parameter("drone_id", 1);
  // DESIGN: waypoints is a flat list [x0,y0,z0, x1,y1,z1, ...].
  // ROS 2 does not support nested lists as parameters, so we flatten triplets.
  declare_parameter("waypoints", std::vector<double>{});
  declare_parameter("waypoint_radius_m", 0.5);
  declare_parameter("republish_rate_hz", 1.0);

  const int drone_id = static_cast<int>(get_parameter("drone_id").as_int());
  const auto raw_wps = get_parameter("waypoints").as_double_array();
  const double radius_m = get_parameter("waypoint_radius_m").as_double();
  const double rate_hz = get_parameter("republish_rate_hz").as_double();

  if (raw_wps.size() % 3 != 0) {
    RCLCPP_WARN(get_logger(),
                "waypoints length (%zu) is not a multiple of 3 — "
                "ignoring the last incomplete triplet",
                raw_wps.size());
  }

  std::vector<Waypoint> waypoints;
  for (size_t i = 0; i + 3 <= raw_wps.size(); i += 3) {
    waypoints.push_back({raw_wps[i], raw_wps[i + 1], raw_wps[i + 2]});
  }

  manager_ = std::make_unique<WaypointManager>(std::move(waypoints), radius_m);

  const std::string ns = "/drone_" + std::to_string(drone_id);

  // DESIGN: Pose comes from sensors → best_effort. Waypoint commands are
  // authoritative → reliable. See CLAUDE.md QoS section.
  auto pose_qos =
      rclcpp::QoS(rclcpp::KeepLast(10))
          .reliability(rclcpp::ReliabilityPolicy::BestEffort);

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      ns + "/pose/estimated", pose_qos,
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
        pose_callback(msg);
      });

  auto cmd_qos =
      rclcpp::QoS(rclcpp::KeepLast(1))
          .reliability(rclcpp::ReliabilityPolicy::Reliable);

  waypoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      ns + "/mission/waypoint", cmd_qos);

  auto period = std::chrono::duration<double>(1.0 / rate_hz);
  republish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { republish_timer_callback(); });

  RCLCPP_INFO(get_logger(),
              "MissionController started: drone_id=%d, %zu waypoints, "
              "radius=%.2f m",
              drone_id, manager_->waypoints_remaining(), radius_m);

  if (!manager_->has_active_waypoint()) {
    RCLCPP_WARN(get_logger(), "No waypoints loaded — mission controller idle");
  }
}

void MissionControllerNode::pose_callback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
  const auto& pos = msg->pose.position;
  const bool advanced = manager_->update_pose(pos.x, pos.y, pos.z);
  if (advanced) {
    if (manager_->has_active_waypoint()) {
      RCLCPP_INFO(get_logger(), "Waypoint reached — advancing to waypoint %zu",
                  manager_->current_index());
    } else {
      RCLCPP_INFO(get_logger(), "All waypoints completed");
    }
    publish_current_waypoint();
  }
}

void MissionControllerNode::republish_timer_callback() {
  publish_current_waypoint();
}

void MissionControllerNode::publish_current_waypoint() {
  if (!manager_->has_active_waypoint()) {
    return;
  }
  const auto& wp = manager_->current_waypoint();
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "map";
  msg.pose.position.x = wp.x;
  msg.pose.position.y = wp.y;
  msg.pose.position.z = wp.z;
  // DESIGN: Identity quaternion — mission controller commands XYZ position
  // only; heading is managed by the flight controller.
  msg.pose.orientation.w = 1.0;
  waypoint_pub_->publish(msg);
}

}  // namespace mission_controller
