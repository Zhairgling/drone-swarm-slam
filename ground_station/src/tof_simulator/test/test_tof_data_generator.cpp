#include "tof_simulator/tof_data_generator.hpp"

#include <cmath>
#include <cstring>
#include <numeric>

#include <gtest/gtest.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace tof_simulator {
namespace {

SimulatorConfig make_test_config() {
  SimulatorConfig config;
  config.drone_id = 1;
  config.fov_deg = 45.0;
  config.max_range_mm = 4000.0;
  config.noise_stddev_mm = 0.0;  // No noise for deterministic tests
  config.room = {5.0, 5.0, 3.0};
  config.drone_position = {2.5, 2.5, 1.5};
  config.sensor_poses = default_sensor_poses();
  return config;
}

// --- default_sensor_poses ---

TEST(DefaultSensorPoses, ReturnsFourSensors) {
  auto poses = default_sensor_poses();
  EXPECT_EQ(poses.size(), 4u);
}

TEST(DefaultSensorPoses, FrontSensorFacesPositiveX) {
  auto poses = default_sensor_poses();
  EXPECT_NEAR(poses[0].yaw, 0.0, 1e-9);
  EXPECT_NEAR(poses[0].pitch, 0.0, 1e-9);
  EXPECT_GT(poses[0].x, 0.0);
}

TEST(DefaultSensorPoses, BackSensorFacesNegativeX) {
  auto poses = default_sensor_poses();
  EXPECT_NEAR(poses[1].yaw, M_PI, 1e-9);
  EXPECT_LT(poses[1].x, 0.0);
}

TEST(DefaultSensorPoses, LeftSensorFacesPositiveY) {
  auto poses = default_sensor_poses();
  EXPECT_NEAR(poses[2].yaw, M_PI / 2.0, 1e-9);
  EXPECT_GT(poses[2].y, 0.0);
}

TEST(DefaultSensorPoses, RightSensorFacesNegativeY) {
  auto poses = default_sensor_poses();
  EXPECT_NEAR(poses[3].yaw, -M_PI / 2.0, 1e-9);
  EXPECT_LT(poses[3].y, 0.0);
}

// --- ray_box_distance ---

TEST(RayBoxDistance, RayAlongPositiveXHitsWall) {
  Point3d origin = {2.5, 2.5, 1.5};
  Point3d direction = {1.0, 0.0, 0.0};
  RoomConfig room = {5.0, 5.0, 3.0};
  double dist = TofDataGenerator::ray_box_distance(origin, direction, room, 10.0);
  EXPECT_NEAR(dist, 2.5, 1e-6);
}

TEST(RayBoxDistance, RayAlongNegativeXHitsWall) {
  Point3d origin = {2.5, 2.5, 1.5};
  Point3d direction = {-1.0, 0.0, 0.0};
  RoomConfig room = {5.0, 5.0, 3.0};
  double dist = TofDataGenerator::ray_box_distance(origin, direction, room, 10.0);
  EXPECT_NEAR(dist, 2.5, 1e-6);
}

TEST(RayBoxDistance, RayAlongPositiveYHitsWall) {
  Point3d origin = {2.5, 2.5, 1.5};
  Point3d direction = {0.0, 1.0, 0.0};
  RoomConfig room = {5.0, 5.0, 3.0};
  double dist = TofDataGenerator::ray_box_distance(origin, direction, room, 10.0);
  EXPECT_NEAR(dist, 2.5, 1e-6);
}

TEST(RayBoxDistance, RayDownHitsFloor) {
  Point3d origin = {2.5, 2.5, 1.5};
  Point3d direction = {0.0, 0.0, -1.0};
  RoomConfig room = {5.0, 5.0, 3.0};
  double dist = TofDataGenerator::ray_box_distance(origin, direction, room, 10.0);
  EXPECT_NEAR(dist, 1.5, 1e-6);
}

TEST(RayBoxDistance, RayUpHitsCeiling) {
  Point3d origin = {2.5, 2.5, 1.5};
  Point3d direction = {0.0, 0.0, 1.0};
  RoomConfig room = {5.0, 5.0, 3.0};
  double dist = TofDataGenerator::ray_box_distance(origin, direction, room, 10.0);
  EXPECT_NEAR(dist, 1.5, 1e-6);
}

TEST(RayBoxDistance, DiagonalRayHitsNearestWall) {
  Point3d origin = {2.5, 2.5, 1.5};
  Point3d direction = {1.0, 1.0, 0.0};  // 45 degrees in XY
  RoomConfig room = {5.0, 5.0, 3.0};
  double dist = TofDataGenerator::ray_box_distance(origin, direction, room, 10.0);
  // At 45 degrees, hits corner at (5.0, 5.0): distance = 2.5 * sqrt(2)
  EXPECT_NEAR(dist, 2.5 * std::sqrt(2.0), 1e-6);
}

TEST(RayBoxDistance, AsymmetricPositionHitsShorterWall) {
  Point3d origin = {1.0, 2.5, 1.5};
  Point3d direction = {-1.0, 0.0, 0.0};
  RoomConfig room = {5.0, 5.0, 3.0};
  double dist = TofDataGenerator::ray_box_distance(origin, direction, room, 10.0);
  EXPECT_NEAR(dist, 1.0, 1e-6);
}

TEST(RayBoxDistance, ReturnsMaxDistanceWhenNoHit) {
  // This shouldn't happen inside a room, but test with zero-sized direction
  Point3d origin = {2.5, 2.5, 1.5};
  Point3d direction = {0.0, 0.0, 0.0};
  RoomConfig room = {5.0, 5.0, 3.0};
  double dist = TofDataGenerator::ray_box_distance(origin, direction, room, 10.0);
  EXPECT_NEAR(dist, 10.0, 1e-6);
}

// --- generate_ranges ---

TEST(GenerateRanges, ProducesCorrectNumberOfRanges) {
  auto config = make_test_config();
  TofDataGenerator gen(config, 42);
  auto ranges = gen.generate_ranges(0);
  EXPECT_EQ(ranges.size(), static_cast<size_t>(kZonesPerSensor));
}

TEST(GenerateRanges, RangesWithinValidBounds) {
  auto config = make_test_config();
  TofDataGenerator gen(config, 42);

  for (int s = 0; s < static_cast<int>(config.sensor_poses.size()); ++s) {
    auto ranges = gen.generate_ranges(s);
    for (double r : ranges) {
      EXPECT_GE(r, 0.0) << "sensor=" << s;
      EXPECT_LE(r, config.max_range_mm) << "sensor=" << s;
    }
  }
}

TEST(GenerateRanges, FrontSensorCenterZoneMeasuresDistanceToWall) {
  auto config = make_test_config();
  config.noise_stddev_mm = 0.0;
  TofDataGenerator gen(config, 42);

  auto ranges = gen.generate_ranges(0);  // front sensor
  // Center zones (3,3) (3,4) (4,3) (4,4) should measure ~2.45m to front wall
  // (drone at 2.5m from front wall, sensor offset 0.05m forward)
  double center_range = ranges[3 * kZonesPerAxis + 3];
  double expected_mm = (2.5 - 0.05) * 1000.0;  // ~2450mm
  EXPECT_NEAR(center_range, expected_mm, 50.0);
}

TEST(GenerateRanges, NoiseIncreasesVariance) {
  SimulatorConfig config_quiet = make_test_config();
  config_quiet.noise_stddev_mm = 0.0;

  SimulatorConfig config_noisy = make_test_config();
  config_noisy.noise_stddev_mm = 50.0;

  TofDataGenerator gen_quiet(config_quiet, 42);
  TofDataGenerator gen_noisy(config_noisy, 42);

  auto ranges_quiet = gen_quiet.generate_ranges(0);
  auto ranges_noisy = gen_noisy.generate_ranges(0);

  // Compute variance of each set
  auto variance = [](const std::vector<double>& v) {
    double mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    double var = 0.0;
    for (double x : v) var += (x - mean) * (x - mean);
    return var / v.size();
  };

  // Noisy ranges should have more variance (noise adds spread)
  EXPECT_GT(variance(ranges_noisy), variance(ranges_quiet));
}

TEST(GenerateRanges, DeterministicWithSameSeed) {
  auto config = make_test_config();
  config.noise_stddev_mm = 20.0;

  TofDataGenerator gen1(config, 123);
  TofDataGenerator gen2(config, 123);

  auto ranges1 = gen1.generate_ranges(0);
  auto ranges2 = gen2.generate_ranges(0);

  ASSERT_EQ(ranges1.size(), ranges2.size());
  for (size_t i = 0; i < ranges1.size(); ++i) {
    EXPECT_DOUBLE_EQ(ranges1[i], ranges2[i]);
  }
}

// --- ranges_to_points ---

TEST(RangesToPoints, ProducesPointsForAllValidRanges) {
  SensorPose pose = {0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> ranges(kZonesPerSensor, 1000.0);
  auto points = TofDataGenerator::ranges_to_points(ranges, pose, 45.0);
  EXPECT_EQ(points.size(), static_cast<size_t>(kZonesPerSensor));
}

TEST(RangesToPoints, ZeroRangesProduceNoPoints) {
  SensorPose pose = {0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> ranges(kZonesPerSensor, 0.0);
  auto points = TofDataGenerator::ranges_to_points(ranges, pose, 45.0);
  EXPECT_EQ(points.size(), 0u);
}

TEST(RangesToPoints, FrontSensorPointsHavePositiveX) {
  SensorPose pose = {0.05, 0.0, 0.0, 0.0, 0.0};  // front sensor
  std::vector<double> ranges(kZonesPerSensor, 2000.0);
  auto points = TofDataGenerator::ranges_to_points(ranges, pose, 45.0);

  for (const auto& pt : points) {
    EXPECT_GT(pt.x, 0.0) << "Front sensor points should have positive X";
  }
}

TEST(RangesToPoints, BackSensorPointsHaveNegativeX) {
  SensorPose pose = {-0.05, 0.0, 0.0, M_PI, 0.0};  // back sensor
  std::vector<double> ranges(kZonesPerSensor, 2000.0);
  auto points = TofDataGenerator::ranges_to_points(ranges, pose, 45.0);

  for (const auto& pt : points) {
    EXPECT_LT(pt.x, 0.0) << "Back sensor points should have negative X";
  }
}

TEST(RangesToPoints, CenterZonePointAlongBoresight) {
  // With no yaw/pitch and uniform range, the center-ish zones should be
  // roughly along +X for a front sensor
  SensorPose pose = {0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> ranges(kZonesPerSensor, 1000.0);  // 1m
  auto points = TofDataGenerator::ranges_to_points(ranges, pose, 45.0);

  // Zone (3,3) is near center - should be approximately (1, 0, 0)
  auto pt = points[3 * kZonesPerAxis + 3];
  EXPECT_NEAR(pt.x, 1.0, 0.1);
  EXPECT_NEAR(pt.y, 0.0, 0.1);
  EXPECT_NEAR(pt.z, 0.0, 0.1);
}

TEST(RangesToPoints, SensorOffsetApplied) {
  // Sensor at offset (0.1, 0.2, 0.3), facing +X, 1m range
  SensorPose pose = {0.1, 0.2, 0.3, 0.0, 0.0};
  std::vector<double> ranges(kZonesPerSensor, 1000.0);
  auto points = TofDataGenerator::ranges_to_points(ranges, pose, 45.0);

  // Center zone should be at approx (1.1, 0.2, 0.3)
  auto pt = points[3 * kZonesPerAxis + 3];
  EXPECT_NEAR(pt.x, 1.1, 0.1);
  EXPECT_NEAR(pt.y, 0.2, 0.1);
  EXPECT_NEAR(pt.z, 0.3, 0.1);
}

// --- generate_pointcloud ---

TEST(GeneratePointcloud, HasCorrectHeader) {
  auto config = make_test_config();
  TofDataGenerator gen(config, 42);
  auto cloud = gen.generate_pointcloud("drone_1/base_link", 100, 500);

  EXPECT_EQ(cloud.header.frame_id, "drone_1/base_link");
  EXPECT_EQ(cloud.header.stamp.sec, 100);
  EXPECT_EQ(cloud.header.stamp.nanosec, 500u);
}

TEST(GeneratePointcloud, HasCorrectDimensions) {
  auto config = make_test_config();
  TofDataGenerator gen(config, 42);
  auto cloud = gen.generate_pointcloud("test", 0, 0);

  EXPECT_EQ(cloud.height, 1u);
  // 4 sensors * 64 zones = 256 points (may be slightly fewer if any range is 0)
  EXPECT_GT(cloud.width, 0u);
  EXPECT_LE(cloud.width, static_cast<uint32_t>(gen.points_per_frame()));
}

TEST(GeneratePointcloud, HasXYZFields) {
  auto config = make_test_config();
  TofDataGenerator gen(config, 42);
  auto cloud = gen.generate_pointcloud("test", 0, 0);

  ASSERT_EQ(cloud.fields.size(), 3u);
  EXPECT_EQ(cloud.fields[0].name, "x");
  EXPECT_EQ(cloud.fields[1].name, "y");
  EXPECT_EQ(cloud.fields[2].name, "z");

  for (const auto& f : cloud.fields) {
    EXPECT_EQ(f.datatype, sensor_msgs::msg::PointField::FLOAT32);
    EXPECT_EQ(f.count, 1u);
  }
}

TEST(GeneratePointcloud, PointStepIs12Bytes) {
  auto config = make_test_config();
  TofDataGenerator gen(config, 42);
  auto cloud = gen.generate_pointcloud("test", 0, 0);

  EXPECT_EQ(cloud.point_step, 12u);  // 3 * float32
}

TEST(GeneratePointcloud, DataSizeMatchesDimensions) {
  auto config = make_test_config();
  TofDataGenerator gen(config, 42);
  auto cloud = gen.generate_pointcloud("test", 0, 0);

  EXPECT_EQ(cloud.data.size(), cloud.width * cloud.point_step);
  EXPECT_EQ(cloud.row_step, cloud.width * cloud.point_step);
}

TEST(GeneratePointcloud, PointsAreFinite) {
  auto config = make_test_config();
  TofDataGenerator gen(config, 42);
  auto cloud = gen.generate_pointcloud("test", 0, 0);

  for (uint32_t i = 0; i < cloud.width; ++i) {
    float x, y, z;
    std::memcpy(&x, &cloud.data[i * 12], 4);
    std::memcpy(&y, &cloud.data[i * 12 + 4], 4);
    std::memcpy(&z, &cloud.data[i * 12 + 8], 4);
    EXPECT_TRUE(std::isfinite(x)) << "Point " << i;
    EXPECT_TRUE(std::isfinite(y)) << "Point " << i;
    EXPECT_TRUE(std::isfinite(z)) << "Point " << i;
  }
}

TEST(GeneratePointcloud, IsDenseTrue) {
  auto config = make_test_config();
  TofDataGenerator gen(config, 42);
  auto cloud = gen.generate_pointcloud("test", 0, 0);
  EXPECT_TRUE(cloud.is_dense);
}

TEST(GeneratePointcloud, IsNotBigEndian) {
  auto config = make_test_config();
  TofDataGenerator gen(config, 42);
  auto cloud = gen.generate_pointcloud("test", 0, 0);
  EXPECT_FALSE(cloud.is_bigendian);
}

// --- points_per_frame ---

TEST(PointsPerFrame, DefaultConfigGives256) {
  auto config = make_test_config();
  TofDataGenerator gen(config, 42);
  EXPECT_EQ(gen.points_per_frame(), 4 * kZonesPerSensor);  // 4 * 64 = 256
}

TEST(PointsPerFrame, SingleSensorGives64) {
  auto config = make_test_config();
  config.sensor_poses = {{0.0, 0.0, 0.0, 0.0, 0.0}};
  TofDataGenerator gen(config, 42);
  EXPECT_EQ(gen.points_per_frame(), kZonesPerSensor);  // 64
}

// --- config accessor ---

TEST(Config, ReturnsConfiguredValues) {
  SimulatorConfig config;
  config.drone_id = 7;
  config.max_range_mm = 2000.0;
  config.sensor_poses = default_sensor_poses();
  TofDataGenerator gen(config, 42);

  EXPECT_EQ(gen.config().drone_id, 7);
  EXPECT_DOUBLE_EQ(gen.config().max_range_mm, 2000.0);
}

TEST(Config, EmptyPosesGetDefaults) {
  SimulatorConfig config;
  config.sensor_poses.clear();
  TofDataGenerator gen(config, 42);

  EXPECT_EQ(gen.config().sensor_poses.size(), 4u);
}

}  // namespace
}  // namespace tof_simulator
