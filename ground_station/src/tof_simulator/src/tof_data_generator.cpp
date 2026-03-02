#include "tof_simulator/tof_data_generator.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include <sensor_msgs/msg/point_field.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace tof_simulator {

std::vector<SensorPose> default_sensor_poses() {
  // DESIGN: 4 sensors at slight offsets from center, facing outward.
  // Matches typical drone mount: front/back/left/right for 360° horizontal FoV.
  constexpr double kOffset = 0.05;  // 5cm from center
  return {
      {kOffset, 0.0, 0.0, 0.0, 0.0},               // front (+X)
      {-kOffset, 0.0, 0.0, M_PI, 0.0},              // back (-X)
      {0.0, kOffset, 0.0, M_PI / 2.0, 0.0},         // left (+Y)
      {0.0, -kOffset, 0.0, -M_PI / 2.0, 0.0},       // right (-Y)
  };
}

TofDataGenerator::TofDataGenerator(const SimulatorConfig& config)
    : config_(config), rng_(std::random_device{}()) {
  if (config_.sensor_poses.empty()) {
    config_.sensor_poses = default_sensor_poses();
  }
}

TofDataGenerator::TofDataGenerator(const SimulatorConfig& config, uint32_t seed)
    : config_(config), rng_(seed) {
  if (config_.sensor_poses.empty()) {
    config_.sensor_poses = default_sensor_poses();
  }
}

int TofDataGenerator::points_per_frame() const {
  return static_cast<int>(config_.sensor_poses.size()) * kZonesPerSensor;
}

double TofDataGenerator::ray_box_distance(const Point3d& origin,
                                          const Point3d& direction,
                                          const RoomConfig& room,
                                          double max_distance) {
  double min_t = max_distance;

  // Check intersection with each of the 6 axis-aligned planes
  // X planes: x=0 and x=room.size_x
  if (std::abs(direction.x) > 1e-9) {
    for (double plane_x : {0.0, room.size_x}) {
      double t = (plane_x - origin.x) / direction.x;
      if (t > 1e-6 && t < min_t) {
        double y = origin.y + t * direction.y;
        double z = origin.z + t * direction.z;
        if (y >= 0.0 && y <= room.size_y && z >= 0.0 && z <= room.size_z) {
          min_t = t;
        }
      }
    }
  }

  // Y planes: y=0 and y=room.size_y
  if (std::abs(direction.y) > 1e-9) {
    for (double plane_y : {0.0, room.size_y}) {
      double t = (plane_y - origin.y) / direction.y;
      if (t > 1e-6 && t < min_t) {
        double y_check = origin.x + t * direction.x;
        double z = origin.z + t * direction.z;
        if (y_check >= 0.0 && y_check <= room.size_x && z >= 0.0 &&
            z <= room.size_z) {
          min_t = t;
        }
      }
    }
  }

  // Z planes: z=0 (floor) and z=room.size_z (ceiling)
  if (std::abs(direction.z) > 1e-9) {
    for (double plane_z : {0.0, room.size_z}) {
      double t = (plane_z - origin.z) / direction.z;
      if (t > 1e-6 && t < min_t) {
        double x = origin.x + t * direction.x;
        double y = origin.y + t * direction.y;
        if (x >= 0.0 && x <= room.size_x && y >= 0.0 && y <= room.size_y) {
          min_t = t;
        }
      }
    }
  }

  return min_t;
}

std::vector<double> TofDataGenerator::generate_ranges(int sensor_index) const {
  const auto& pose = config_.sensor_poses.at(sensor_index);
  const double zone_angle_rad =
      (config_.fov_deg / kZonesPerAxis) * M_PI / 180.0;

  // Precompute rotation matrix: R = Rz(yaw) * Ry(pitch)
  const double cy = std::cos(pose.yaw);
  const double sy = std::sin(pose.yaw);
  const double cp = std::cos(pose.pitch);
  const double sp = std::sin(pose.pitch);

  // Sensor origin in world frame
  Point3d sensor_origin = {
      config_.drone_position.x + pose.x,
      config_.drone_position.y + pose.y,
      config_.drone_position.z + pose.z,
  };

  std::normal_distribution<double> noise(0.0, config_.noise_stddev_mm);
  const double max_range_m = config_.max_range_mm / 1000.0;

  std::vector<double> ranges(kZonesPerSensor);

  for (int row = 0; row < kZonesPerAxis; ++row) {
    for (int col = 0; col < kZonesPerAxis; ++col) {
      // Zone angles relative to sensor boresight
      double h_angle = (col - 3.5) * zone_angle_rad;
      double v_angle = -(row - 3.5) * zone_angle_rad;

      // Ray direction in sensor local frame (+X forward)
      double dx_local = std::cos(v_angle) * std::cos(h_angle);
      double dy_local = std::cos(v_angle) * std::sin(h_angle);
      double dz_local = std::sin(v_angle);

      // Rotate to drone/world frame: R = Rz(yaw) * Ry(pitch)
      Point3d dir = {
          cy * cp * dx_local - sy * dy_local + cy * sp * dz_local,
          sy * cp * dx_local + cy * dy_local + sy * sp * dz_local,
          -sp * dx_local + cp * dz_local,
      };

      double distance_m =
          ray_box_distance(sensor_origin, dir, config_.room, max_range_m);
      double distance_mm = distance_m * 1000.0;

      // Add noise and clamp
      distance_mm += noise(rng_);
      distance_mm = std::clamp(distance_mm, 0.0, config_.max_range_mm);

      ranges[row * kZonesPerAxis + col] = distance_mm;
    }
  }

  return ranges;
}

std::vector<Point3d> TofDataGenerator::ranges_to_points(
    const std::vector<double>& ranges_mm, const SensorPose& pose,
    double fov_deg) {
  const double zone_angle_rad = (fov_deg / kZonesPerAxis) * M_PI / 180.0;

  const double cy = std::cos(pose.yaw);
  const double sy = std::sin(pose.yaw);
  const double cp = std::cos(pose.pitch);
  const double sp = std::sin(pose.pitch);

  std::vector<Point3d> points;
  points.reserve(ranges_mm.size());

  for (int row = 0; row < kZonesPerAxis; ++row) {
    for (int col = 0; col < kZonesPerAxis; ++col) {
      int idx = row * kZonesPerAxis + col;
      if (idx >= static_cast<int>(ranges_mm.size())) break;

      double range_m = ranges_mm[idx] / 1000.0;
      if (range_m <= 0.0) continue;

      double h_angle = (col - 3.5) * zone_angle_rad;
      double v_angle = -(row - 3.5) * zone_angle_rad;

      double dx_local = std::cos(v_angle) * std::cos(h_angle) * range_m;
      double dy_local = std::cos(v_angle) * std::sin(h_angle) * range_m;
      double dz_local = std::sin(v_angle) * range_m;

      // Rotate to drone frame + offset by sensor position
      Point3d pt = {
          cy * cp * dx_local - sy * dy_local + cy * sp * dz_local + pose.x,
          sy * cp * dx_local + cy * dy_local + sy * sp * dz_local + pose.y,
          -sp * dx_local + cp * dz_local + pose.z,
      };

      points.push_back(pt);
    }
  }

  return points;
}

sensor_msgs::msg::PointCloud2 TofDataGenerator::generate_pointcloud(
    const std::string& frame_id, int32_t stamp_sec,
    uint32_t stamp_nanosec) const {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp.sec = stamp_sec;
  cloud.header.stamp.nanosec = stamp_nanosec;

  // Collect all points from all sensors
  std::vector<Point3d> all_points;
  all_points.reserve(points_per_frame());

  for (int s = 0; s < static_cast<int>(config_.sensor_poses.size()); ++s) {
    auto ranges = generate_ranges(s);
    auto pts = ranges_to_points(ranges, config_.sensor_poses[s], config_.fov_deg);
    all_points.insert(all_points.end(), pts.begin(), pts.end());
  }

  // PointCloud2 field layout: x(float32), y(float32), z(float32)
  constexpr uint32_t kFloat32Size = 4;
  constexpr uint32_t kPointStep = kFloat32Size * 3;

  sensor_msgs::msg::PointField field_x;
  field_x.name = "x";
  field_x.offset = 0;
  field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_x.count = 1;

  sensor_msgs::msg::PointField field_y;
  field_y.name = "y";
  field_y.offset = kFloat32Size;
  field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_y.count = 1;

  sensor_msgs::msg::PointField field_z;
  field_z.name = "z";
  field_z.offset = kFloat32Size * 2;
  field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_z.count = 1;

  cloud.fields = {field_x, field_y, field_z};
  cloud.is_bigendian = false;
  cloud.point_step = kPointStep;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(all_points.size());
  cloud.row_step = cloud.width * kPointStep;
  cloud.is_dense = true;

  cloud.data.resize(all_points.size() * kPointStep);
  for (size_t i = 0; i < all_points.size(); ++i) {
    auto x = static_cast<float>(all_points[i].x);
    auto y = static_cast<float>(all_points[i].y);
    auto z = static_cast<float>(all_points[i].z);
    std::memcpy(&cloud.data[i * kPointStep], &x, kFloat32Size);
    std::memcpy(&cloud.data[i * kPointStep + kFloat32Size], &y, kFloat32Size);
    std::memcpy(&cloud.data[i * kPointStep + kFloat32Size * 2], &z,
                kFloat32Size);
  }

  return cloud;
}

}  // namespace tof_simulator
