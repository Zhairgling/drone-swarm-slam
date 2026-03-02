// cppcheck-suppress-file syntaxError
#include "pointcloud_assembler/pointcloud_assembler.hpp"

#include <cstring>

#include <gtest/gtest.h>

#include <sensor_msgs/msg/point_field.hpp>

namespace pointcloud_assembler {
namespace {

/// Build a minimal x/y/z float32 PointCloud2 with num_points points.
sensor_msgs::msg::PointCloud2 make_cloud(int32_t sec, uint32_t nanosec,
                                         uint32_t num_points) {
  constexpr uint32_t kFloat32Size = 4;
  constexpr uint32_t kPointStep = kFloat32Size * 3;

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp.sec = sec;
  cloud.header.stamp.nanosec = nanosec;
  cloud.header.frame_id = "drone_1/base_link";

  sensor_msgs::msg::PointField fx;
  fx.name = "x";
  fx.offset = 0;
  fx.datatype = sensor_msgs::msg::PointField::FLOAT32;
  fx.count = 1;

  sensor_msgs::msg::PointField fy;
  fy.name = "y";
  fy.offset = kFloat32Size;
  fy.datatype = sensor_msgs::msg::PointField::FLOAT32;
  fy.count = 1;

  sensor_msgs::msg::PointField fz;
  fz.name = "z";
  fz.offset = kFloat32Size * 2;
  fz.datatype = sensor_msgs::msg::PointField::FLOAT32;
  fz.count = 1;

  cloud.fields = {fx, fy, fz};
  cloud.point_step = kPointStep;
  cloud.height = 1;
  cloud.width = num_points;
  cloud.row_step = num_points * kPointStep;
  cloud.is_dense = true;
  cloud.is_bigendian = false;

  cloud.data.resize(num_points * kPointStep, 0);
  // Write point index as x value for traceability in tests.
  for (uint32_t i = 0; i < num_points; ++i) {
    auto val = static_cast<float>(i);
    std::memcpy(cloud.data.data() + i * kPointStep, &val, kFloat32Size);
  }

  return cloud;
}

// --- add_cloud ---

TEST(AddCloud, BufferSizeIncrements) {
  AssemblerConfig cfg;
  PointcloudAssembler a(cfg);

  EXPECT_EQ(a.buffer_size(), 0u);
  a.add_cloud(make_cloud(1, 0, 10));
  EXPECT_EQ(a.buffer_size(), 1u);
  a.add_cloud(make_cloud(2, 0, 10));
  EXPECT_EQ(a.buffer_size(), 2u);
}

TEST(AddCloud, TotalPointsAccumulates) {
  AssemblerConfig cfg;
  PointcloudAssembler a(cfg);

  a.add_cloud(make_cloud(1, 0, 10));
  EXPECT_EQ(a.total_points(), 10u);
  a.add_cloud(make_cloud(2, 0, 20));
  EXPECT_EQ(a.total_points(), 30u);
}

TEST(AddCloud, EvictsOldestWhenAtCapacity) {
  AssemblerConfig cfg;
  cfg.max_clouds = 3;
  PointcloudAssembler a(cfg);

  a.add_cloud(make_cloud(1, 0, 5));
  a.add_cloud(make_cloud(2, 0, 5));
  a.add_cloud(make_cloud(3, 0, 5));
  EXPECT_EQ(a.buffer_size(), 3u);

  // 4th cloud should evict the first.
  a.add_cloud(make_cloud(4, 0, 5));
  EXPECT_EQ(a.buffer_size(), 3u);
  EXPECT_EQ(a.total_points(), 15u);  // 3 clouds × 5 pts
}

TEST(AddCloud, EmptyCloudAccepted) {
  AssemblerConfig cfg;
  PointcloudAssembler a(cfg);
  a.add_cloud(make_cloud(0, 0, 0));
  EXPECT_EQ(a.buffer_size(), 1u);
  EXPECT_EQ(a.total_points(), 0u);
}

// --- expire_old_clouds ---

TEST(ExpireOldClouds, RemovesCloudsOlderThanWindow) {
  AssemblerConfig cfg;
  cfg.window_duration_sec = 2.0;
  PointcloudAssembler a(cfg);

  // stamp at t=1s, t=2s, t=5s
  a.add_cloud(make_cloud(1, 0, 10));
  a.add_cloud(make_cloud(2, 0, 10));
  a.add_cloud(make_cloud(5, 0, 10));

  // Reference at t=6s → window covers [4s, 6s] → expire anything < 4s.
  int64_t ref_ns = 6LL * 1000000000LL;
  a.expire_old_clouds(ref_ns);

  // Only the t=5s cloud remains.
  EXPECT_EQ(a.buffer_size(), 1u);
  EXPECT_EQ(a.total_points(), 10u);
}

TEST(ExpireOldClouds, KeepsAllCloudsWithinWindow) {
  AssemblerConfig cfg;
  cfg.window_duration_sec = 10.0;
  PointcloudAssembler a(cfg);

  a.add_cloud(make_cloud(1, 0, 5));
  a.add_cloud(make_cloud(2, 0, 5));
  a.add_cloud(make_cloud(3, 0, 5));

  int64_t ref_ns = 5LL * 1000000000LL;  // all clouds within 10s window
  a.expire_old_clouds(ref_ns);

  EXPECT_EQ(a.buffer_size(), 3u);
}

TEST(ExpireOldClouds, EmptyBufferIsNoop) {
  AssemblerConfig cfg;
  PointcloudAssembler a(cfg);
  a.expire_old_clouds(10LL * 1000000000LL);  // should not crash
  EXPECT_EQ(a.buffer_size(), 0u);
}

TEST(ExpireOldClouds, ExpiresAllClouds) {
  AssemblerConfig cfg;
  cfg.window_duration_sec = 1.0;
  PointcloudAssembler a(cfg);

  a.add_cloud(make_cloud(1, 0, 10));
  a.add_cloud(make_cloud(2, 0, 10));

  // Reference far in the future → all clouds expire.
  int64_t ref_ns = 1000LL * 1000000000LL;
  a.expire_old_clouds(ref_ns);

  EXPECT_EQ(a.buffer_size(), 0u);
  EXPECT_EQ(a.total_points(), 0u);
}

// --- assemble ---

TEST(Assemble, EmptyBufferReturnsEmptyCloud) {
  AssemblerConfig cfg;
  PointcloudAssembler a(cfg);

  auto cloud = a.assemble("odom", 0, 0);
  EXPECT_EQ(cloud.width, 0u);
  EXPECT_EQ(cloud.height, 1u);
  EXPECT_TRUE(cloud.data.empty());
}

TEST(Assemble, SingleCloudCopiesData) {
  AssemblerConfig cfg;
  PointcloudAssembler a(cfg);

  a.add_cloud(make_cloud(1, 0, 16));
  auto cloud = a.assemble("odom", 1, 0);

  EXPECT_EQ(cloud.width, 16u);
  EXPECT_EQ(cloud.height, 1u);
  EXPECT_EQ(cloud.data.size(), 16u * 12u);
}

TEST(Assemble, TwoCloudsAreConcatenated) {
  AssemblerConfig cfg;
  PointcloudAssembler a(cfg);

  a.add_cloud(make_cloud(1, 0, 10));
  a.add_cloud(make_cloud(2, 0, 20));
  auto cloud = a.assemble("odom", 2, 0);

  EXPECT_EQ(cloud.width, 30u);
  EXPECT_EQ(cloud.data.size(), 30u * 12u);
}

TEST(Assemble, HeaderIsSet) {
  AssemblerConfig cfg;
  PointcloudAssembler a(cfg);
  a.add_cloud(make_cloud(1, 0, 4));

  auto cloud = a.assemble("map", 42, 500u);
  EXPECT_EQ(cloud.header.frame_id, "map");
  EXPECT_EQ(cloud.header.stamp.sec, 42);
  EXPECT_EQ(cloud.header.stamp.nanosec, 500u);
}

TEST(Assemble, FieldsMatchInput) {
  AssemblerConfig cfg;
  PointcloudAssembler a(cfg);
  a.add_cloud(make_cloud(1, 0, 4));

  auto cloud = a.assemble("odom", 0, 0);
  ASSERT_EQ(cloud.fields.size(), 3u);
  EXPECT_EQ(cloud.fields[0].name, "x");
  EXPECT_EQ(cloud.fields[1].name, "y");
  EXPECT_EQ(cloud.fields[2].name, "z");
}

TEST(Assemble, PointStepIs12) {
  AssemblerConfig cfg;
  PointcloudAssembler a(cfg);
  a.add_cloud(make_cloud(1, 0, 4));

  auto cloud = a.assemble("odom", 0, 0);
  EXPECT_EQ(cloud.point_step, 12u);
}

TEST(Assemble, RowStepMatchesWidthTimesPointStep) {
  AssemblerConfig cfg;
  PointcloudAssembler a(cfg);
  a.add_cloud(make_cloud(1, 0, 8));
  a.add_cloud(make_cloud(2, 0, 8));

  auto cloud = a.assemble("odom", 0, 0);
  EXPECT_EQ(cloud.row_step, cloud.width * cloud.point_step);
}

TEST(Assemble, DataSizeMatchesDimensions) {
  AssemblerConfig cfg;
  PointcloudAssembler a(cfg);
  a.add_cloud(make_cloud(1, 0, 64));

  auto cloud = a.assemble("odom", 0, 0);
  EXPECT_EQ(cloud.data.size(), cloud.width * cloud.point_step);
}

TEST(Assemble, IsNotBigEndian) {
  AssemblerConfig cfg;
  PointcloudAssembler a(cfg);
  a.add_cloud(make_cloud(1, 0, 4));
  auto cloud = a.assemble("odom", 0, 0);
  EXPECT_FALSE(cloud.is_bigendian);
}

TEST(Assemble, IsDense) {
  AssemblerConfig cfg;
  PointcloudAssembler a(cfg);
  a.add_cloud(make_cloud(1, 0, 4));
  auto cloud = a.assemble("odom", 0, 0);
  EXPECT_TRUE(cloud.is_dense);
}

// --- buffer_size ---

TEST(BufferSize, StartsAtZero) {
  AssemblerConfig cfg;
  PointcloudAssembler a(cfg);
  EXPECT_EQ(a.buffer_size(), 0u);
}

// --- total_points ---

TEST(TotalPoints, StartsAtZero) {
  AssemblerConfig cfg;
  PointcloudAssembler a(cfg);
  EXPECT_EQ(a.total_points(), 0u);
}

TEST(TotalPoints, SumsAcrossMultipleClouds) {
  AssemblerConfig cfg;
  PointcloudAssembler a(cfg);
  a.add_cloud(make_cloud(1, 0, 100));
  a.add_cloud(make_cloud(2, 0, 56));
  EXPECT_EQ(a.total_points(), 156u);
}

// --- config accessor ---

TEST(Config, ReturnsConfiguredValues) {
  AssemblerConfig cfg;
  cfg.window_duration_sec = 3.5;
  cfg.max_clouds = 42;
  PointcloudAssembler a(cfg);
  EXPECT_DOUBLE_EQ(a.config().window_duration_sec, 3.5);
  EXPECT_EQ(a.config().max_clouds, 42u);
}

}  // namespace
}  // namespace pointcloud_assembler
