#include "tf_broadcaster/drone_tf_broadcaster.hpp"

#include <cmath>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace tf_broadcaster {

TfConfig default_config() {
  TfConfig cfg;
  // DESIGN: positions and yaw angles mirror tof_simulator default_sensor_poses()
  // so that TF frames are consistent with simulated pointcloud origins.
  // FLU convention: +X=forward, +Y=left, +Z=up.
  cfg.tof_front = {kDefaultSensorOffset, 0.0, 0.0, 0.0};
  cfg.tof_right = {0.0, -kDefaultSensorOffset, 0.0, -M_PI / 2.0};
  cfg.tof_back = {-kDefaultSensorOffset, 0.0, 0.0, M_PI};
  cfg.tof_left = {0.0, kDefaultSensorOffset, 0.0, M_PI / 2.0};
  // DESIGN: camera (OV2640) is mounted on XIAO at drone center;
  // exact position TBD after physical assembly.
  cfg.camera = {0.0, 0.0, 0.0, 0.0};
  return cfg;
}

geometry_msgs::msg::TransformStamped make_transform(
    const std::string& parent_frame, const std::string& child_frame,
    const SensorOffset& offset) {
  geometry_msgs::msg::TransformStamped t;
  // Zero timestamp: TF2 treats t=0 as "always valid" for static transforms.
  t.header.stamp.sec = 0;
  t.header.stamp.nanosec = 0;
  t.header.frame_id = parent_frame;
  t.child_frame_id = child_frame;

  t.transform.translation.x = offset.x;
  t.transform.translation.y = offset.y;
  t.transform.translation.z = offset.z;

  // Quaternion from yaw-only rotation (around Z): q = [0, 0, sin(yaw/2), cos(yaw/2)]
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = std::sin(offset.yaw / 2.0);
  t.transform.rotation.w = std::cos(offset.yaw / 2.0);

  return t;
}

std::vector<geometry_msgs::msg::TransformStamped> build_transforms(
    const TfConfig& config) {
  const std::string base =
      "drone_" + std::to_string(config.drone_id) + "/base_link";
  const std::string ns = "drone_" + std::to_string(config.drone_id) + "/";

  return {
      make_transform(base, ns + "tof_front", config.tof_front),
      make_transform(base, ns + "tof_right", config.tof_right),
      make_transform(base, ns + "tof_back", config.tof_back),
      make_transform(base, ns + "tof_left", config.tof_left),
      make_transform(base, ns + "camera", config.camera),
  };
}

DroneTfBroadcaster::DroneTfBroadcaster(const rclcpp::NodeOptions& options)
    : Node("drone_tf_broadcaster", options) {
  declare_parameter("drone_id", 1);

  declare_parameter("tof_front_x", kDefaultSensorOffset);
  declare_parameter("tof_front_y", 0.0);
  declare_parameter("tof_front_z", 0.0);
  declare_parameter("tof_front_yaw", 0.0);

  declare_parameter("tof_right_x", 0.0);
  declare_parameter("tof_right_y", -kDefaultSensorOffset);
  declare_parameter("tof_right_z", 0.0);
  declare_parameter("tof_right_yaw", -M_PI / 2.0);

  declare_parameter("tof_back_x", -kDefaultSensorOffset);
  declare_parameter("tof_back_y", 0.0);
  declare_parameter("tof_back_z", 0.0);
  declare_parameter("tof_back_yaw", M_PI);

  declare_parameter("tof_left_x", 0.0);
  declare_parameter("tof_left_y", kDefaultSensorOffset);
  declare_parameter("tof_left_z", 0.0);
  declare_parameter("tof_left_yaw", M_PI / 2.0);

  declare_parameter("camera_x", 0.0);
  declare_parameter("camera_y", 0.0);
  declare_parameter("camera_z", 0.0);
  declare_parameter("camera_yaw", 0.0);

  TfConfig config;
  config.drone_id = get_parameter("drone_id").as_int();
  config.tof_front = {
      get_parameter("tof_front_x").as_double(),
      get_parameter("tof_front_y").as_double(),
      get_parameter("tof_front_z").as_double(),
      get_parameter("tof_front_yaw").as_double(),
  };
  config.tof_right = {
      get_parameter("tof_right_x").as_double(),
      get_parameter("tof_right_y").as_double(),
      get_parameter("tof_right_z").as_double(),
      get_parameter("tof_right_yaw").as_double(),
  };
  config.tof_back = {
      get_parameter("tof_back_x").as_double(),
      get_parameter("tof_back_y").as_double(),
      get_parameter("tof_back_z").as_double(),
      get_parameter("tof_back_yaw").as_double(),
  };
  config.tof_left = {
      get_parameter("tof_left_x").as_double(),
      get_parameter("tof_left_y").as_double(),
      get_parameter("tof_left_z").as_double(),
      get_parameter("tof_left_yaw").as_double(),
  };
  config.camera = {
      get_parameter("camera_x").as_double(),
      get_parameter("camera_y").as_double(),
      get_parameter("camera_z").as_double(),
      get_parameter("camera_yaw").as_double(),
  };

  broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
  broadcaster_->sendTransform(build_transforms(config));

  RCLCPP_INFO(get_logger(),
              "Published 5 static TF transforms for drone_%d "
              "(tof_front, tof_right, tof_back, tof_left, camera)",
              config.drone_id);
}

}  // namespace tf_broadcaster
