#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace tof_simulator {

// DESIGN: VL53L8CX has a fixed 8x8 zone array. Compile-time constant enables
// stack allocation and matches the hardware constraint.
constexpr int kZonesPerAxis = 8;
constexpr int kZonesPerSensor = kZonesPerAxis * kZonesPerAxis;
constexpr int kDefaultNumSensors = 4;

// DESIGN: VL53L8CX FoV is ~45 deg per axis in standard mode. Max range 4m.
// Noise ~20mm is typical for indoor ToF at medium range.
constexpr double kDefaultFovDeg = 45.0;
constexpr double kDefaultMaxRangeMm = 4000.0;
constexpr double kDefaultNoiseMm = 20.0;

struct SensorPose {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;    // radians, rotation around Z axis
  double pitch = 0.0;  // radians, rotation around Y axis
};

struct Point3d {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct RoomConfig {
  double size_x = 5.0;  // meters
  double size_y = 5.0;
  double size_z = 3.0;
};

struct SimulatorConfig {
  int drone_id = 1;
  double fov_deg = kDefaultFovDeg;
  double max_range_mm = kDefaultMaxRangeMm;
  double noise_stddev_mm = kDefaultNoiseMm;
  std::vector<SensorPose> sensor_poses;
  RoomConfig room;
  Point3d drone_position = {2.5, 2.5, 1.5};
};

/// Returns default sensor poses: front (+X), back (-X), left (+Y), right (-Y)
std::vector<SensorPose> default_sensor_poses();

/// Generates simulated VL53L8CX ToF data and converts to PointCloud2.
/// Separated from ROS node for testability.
class TofDataGenerator {
 public:
  explicit TofDataGenerator(const SimulatorConfig& config);
  TofDataGenerator(const SimulatorConfig& config, uint32_t seed);

  /// Generate one frame of ToF data as PointCloud2
  sensor_msgs::msg::PointCloud2 generate_pointcloud(
      const std::string& frame_id, int32_t stamp_sec,
      uint32_t stamp_nanosec) const;

  /// Generate raw range measurements (mm) for a single sensor
  std::vector<double> generate_ranges(int sensor_index) const;

  /// Convert 8x8 range grid + sensor pose to 3D points in drone frame
  static std::vector<Point3d> ranges_to_points(
      const std::vector<double>& ranges_mm, const SensorPose& pose,
      double fov_deg);

  /// Cast a ray from origin in direction and return distance to room walls (m).
  /// Returns max_distance if no intersection.
  static double ray_box_distance(const Point3d& origin, const Point3d& direction,
                                 const RoomConfig& room, double max_distance);

  /// Number of points per frame (all sensors combined)
  int points_per_frame() const;

  const SimulatorConfig& config() const { return config_; }

 private:
  SimulatorConfig config_;
  mutable std::mt19937 rng_;
};

}  // namespace tof_simulator
