// cppcheck-suppress-file syntaxError
#include "slam_node/occupancy_map.hpp"

#include <cmath>
#include <cstring>
#include <limits>

#include <gtest/gtest.h>

namespace slam_node {
namespace {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

MapConfig make_default_config() {
  MapConfig cfg;
  cfg.resolution = 0.1;
  cfg.size_x = 10.0;
  cfg.size_y = 10.0;
  cfg.height_min = 0.0;
  cfg.height_max = 3.0;
  return cfg;
}

/// Build a minimal unorganized PointCloud2 with x, y, z float32 fields.
sensor_msgs::msg::PointCloud2 make_cloud(
    const std::vector<std::array<float, 3>>& pts) {
  sensor_msgs::msg::PointCloud2 cloud;

  sensor_msgs::msg::PointField fx;
  fx.name = "x";
  fx.offset = 0U;
  fx.datatype = sensor_msgs::msg::PointField::FLOAT32;
  fx.count = 1U;

  sensor_msgs::msg::PointField fy;
  fy.name = "y";
  fy.offset = 4U;
  fy.datatype = sensor_msgs::msg::PointField::FLOAT32;
  fy.count = 1U;

  sensor_msgs::msg::PointField fz;
  fz.name = "z";
  fz.offset = 8U;
  fz.datatype = sensor_msgs::msg::PointField::FLOAT32;
  fz.count = 1U;

  cloud.fields = {fx, fy, fz};
  cloud.is_bigendian = false;
  cloud.point_step = 12U;
  cloud.height = 1U;
  cloud.width = static_cast<uint32_t>(pts.size());
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.is_dense = true;
  cloud.data.resize(pts.size() * 12U);

  for (size_t i = 0; i < pts.size(); ++i) {
    std::memcpy(&cloud.data[i * 12U], &pts[i][0], 4U);
    std::memcpy(&cloud.data[i * 12U + 4U], &pts[i][1], 4U);
    std::memcpy(&cloud.data[i * 12U + 8U], &pts[i][2], 4U);
  }

  return cloud;
}

// ---------------------------------------------------------------------------
// Constructor / config
// ---------------------------------------------------------------------------

TEST(OccupancyMap, StartsEmpty) {
  OccupancyMap map(make_default_config());
  EXPECT_EQ(map.voxel_count(), 0u);
}

TEST(OccupancyMap, ConfigAccessorReturnsValues) {
  auto cfg = make_default_config();
  cfg.resolution = 0.05;
  cfg.size_x = 15.0;
  OccupancyMap map(cfg);
  EXPECT_DOUBLE_EQ(map.config().resolution, 0.05);
  EXPECT_DOUBLE_EQ(map.config().size_x, 15.0);
}

// ---------------------------------------------------------------------------
// clear()
// ---------------------------------------------------------------------------

TEST(OccupancyMap, ClearResetsVoxelCount) {
  OccupancyMap map(make_default_config());
  auto cloud = make_cloud({{0.0F, 0.0F, 1.0F}});
  map.add_pointcloud(cloud);
  EXPECT_GT(map.voxel_count(), 0u);
  map.clear();
  EXPECT_EQ(map.voxel_count(), 0u);
}

// ---------------------------------------------------------------------------
// add_pointcloud()
// ---------------------------------------------------------------------------

TEST(AddPointcloud, SinglePointIncreasesVoxelCount) {
  OccupancyMap map(make_default_config());
  auto cloud = make_cloud({{0.0F, 0.0F, 1.0F}});
  map.add_pointcloud(cloud);
  EXPECT_EQ(map.voxel_count(), 1u);
}

TEST(AddPointcloud, SameVoxelTwiceCountsOnce) {
  OccupancyMap map(make_default_config());
  // Two points that fall in the same 0.1m voxel
  auto cloud = make_cloud({{0.01F, 0.01F, 1.01F}, {0.02F, 0.02F, 1.02F}});
  map.add_pointcloud(cloud);
  EXPECT_EQ(map.voxel_count(), 1u);
}

TEST(AddPointcloud, TwoDistinctVoxelsCountedSeparately) {
  OccupancyMap map(make_default_config());
  // Points at least 0.1m apart in X — guaranteed different voxels
  auto cloud = make_cloud({{0.05F, 0.0F, 1.0F}, {0.95F, 0.0F, 1.0F}});
  map.add_pointcloud(cloud);
  EXPECT_EQ(map.voxel_count(), 2u);
}

TEST(AddPointcloud, OutOfBoundsPointsIgnored) {
  OccupancyMap map(make_default_config());
  // config: size_x=10, centered → bounds [-5, 5)
  // height_min=0, height_max=3
  auto cloud = make_cloud({
      {100.0F, 0.0F, 1.0F},   // X out of bounds
      {0.0F, 100.0F, 1.0F},   // Y out of bounds
      {0.0F, 0.0F, -1.0F},    // Z below min
      {0.0F, 0.0F, 10.0F},    // Z above max
  });
  map.add_pointcloud(cloud);
  EXPECT_EQ(map.voxel_count(), 0u);
}

TEST(AddPointcloud, NonFinitePointsIgnored) {
  OccupancyMap map(make_default_config());
  auto cloud = make_cloud({
      {std::numeric_limits<float>::quiet_NaN(), 0.0F, 1.0F},
      {std::numeric_limits<float>::infinity(), 0.0F, 1.0F},
      {0.0F, 0.0F, 1.0F},  // valid
  });
  map.add_pointcloud(cloud);
  EXPECT_EQ(map.voxel_count(), 1u);
}

TEST(AddPointcloud, EmptyCloudChangesNothing) {
  OccupancyMap map(make_default_config());
  auto cloud = make_cloud({});
  map.add_pointcloud(cloud);
  EXPECT_EQ(map.voxel_count(), 0u);
}

TEST(AddPointcloud, MultipleCloudsAccumulate) {
  OccupancyMap map(make_default_config());
  auto cloud1 = make_cloud({{0.05F, 0.0F, 1.0F}});
  auto cloud2 = make_cloud({{2.05F, 0.0F, 1.0F}});
  map.add_pointcloud(cloud1);
  map.add_pointcloud(cloud2);
  EXPECT_EQ(map.voxel_count(), 2u);
}

TEST(AddPointcloud, CloudWithMissingFieldsIgnored) {
  OccupancyMap map(make_default_config());

  // Cloud with no fields at all
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1U;
  cloud.width = 1U;
  cloud.point_step = 12U;
  cloud.row_step = 12U;
  cloud.data.resize(12U, 0U);

  map.add_pointcloud(cloud);
  EXPECT_EQ(map.voxel_count(), 0u);
}

// ---------------------------------------------------------------------------
// get_occupancy_grid()
// ---------------------------------------------------------------------------

TEST(GetOccupancyGrid, HeaderSetCorrectly) {
  OccupancyMap map(make_default_config());
  auto grid = map.get_occupancy_grid("map", 42, 500U);
  EXPECT_EQ(grid.header.frame_id, "map");
  EXPECT_EQ(grid.header.stamp.sec, 42);
  EXPECT_EQ(grid.header.stamp.nanosec, 500U);
}

TEST(GetOccupancyGrid, InfoMatchesConfig) {
  auto cfg = make_default_config();
  OccupancyMap map(cfg);
  auto grid = map.get_occupancy_grid("map", 0, 0U);

  EXPECT_FLOAT_EQ(grid.info.resolution, static_cast<float>(cfg.resolution));
  // Grid dimensions: ceil(size / resolution)
  const int expected_nx = static_cast<int>(std::ceil(cfg.size_x / cfg.resolution));
  const int expected_ny = static_cast<int>(std::ceil(cfg.size_y / cfg.resolution));
  EXPECT_EQ(grid.info.width, static_cast<uint32_t>(expected_nx));
  EXPECT_EQ(grid.info.height, static_cast<uint32_t>(expected_ny));
}

TEST(GetOccupancyGrid, OriginCenteredAtMapOrigin) {
  auto cfg = make_default_config();
  OccupancyMap map(cfg);
  auto grid = map.get_occupancy_grid("map", 0, 0U);

  EXPECT_DOUBLE_EQ(grid.info.origin.position.x, -cfg.size_x / 2.0);
  EXPECT_DOUBLE_EQ(grid.info.origin.position.y, -cfg.size_y / 2.0);
}

TEST(GetOccupancyGrid, EmptyMapAllUnknown) {
  OccupancyMap map(make_default_config());
  auto grid = map.get_occupancy_grid("map", 0, 0U);

  for (const int8_t cell : grid.data) {
    EXPECT_EQ(cell, static_cast<int8_t>(-1)) << "Empty map should be all unknown";
  }
}

TEST(GetOccupancyGrid, DataSizeMatchesWidthTimesHeight) {
  OccupancyMap map(make_default_config());
  auto grid = map.get_occupancy_grid("map", 0, 0U);
  EXPECT_EQ(grid.data.size(), static_cast<size_t>(grid.info.width) * grid.info.height);
}

TEST(GetOccupancyGrid, OccupiedPointMarkedAs100) {
  OccupancyMap map(make_default_config());
  // Place a point at (0, 0, 1.0) — map center in XY, valid Z
  auto cloud = make_cloud({{0.05F, 0.05F, 1.0F}});
  map.add_pointcloud(cloud);

  auto grid = map.get_occupancy_grid("map", 0, 0U);

  // At least one occupied cell exists
  bool found_occupied = false;
  for (const int8_t cell : grid.data) {
    if (cell == static_cast<int8_t>(100)) { found_occupied = true; break; }
  }
  EXPECT_TRUE(found_occupied);
}

TEST(GetOccupancyGrid, CellCountAfterAddingPoints) {
  OccupancyMap map(make_default_config());
  // Two points in different XY voxels but same Z column → 2 occupied cells
  auto cloud = make_cloud({
      {0.05F, 0.05F, 1.0F},
      {1.05F, 1.05F, 1.0F},
  });
  map.add_pointcloud(cloud);
  auto grid = map.get_occupancy_grid("map", 0, 0U);

  int occupied_count = 0;
  for (const int8_t cell : grid.data) {
    if (cell == static_cast<int8_t>(100)) { ++occupied_count; }
  }
  EXPECT_EQ(occupied_count, 2);
}

TEST(GetOccupancyGrid, MultipleZLayersSameXYColumnCountOnce) {
  OccupancyMap map(make_default_config());
  // Two points at same XY, different Z → same 2D cell
  auto cloud = make_cloud({
      {0.05F, 0.05F, 0.5F},
      {0.05F, 0.05F, 1.5F},
  });
  map.add_pointcloud(cloud);
  auto grid = map.get_occupancy_grid("map", 0, 0U);

  int occupied_count = 0;
  for (const int8_t cell : grid.data) {
    if (cell == static_cast<int8_t>(100)) { ++occupied_count; }
  }
  EXPECT_EQ(occupied_count, 1);
}

// ---------------------------------------------------------------------------
// get_point_cloud()
// ---------------------------------------------------------------------------

TEST(GetPointCloud, HeaderSetCorrectly) {
  OccupancyMap map(make_default_config());
  auto cloud = map.get_point_cloud("map", 10, 200U);
  EXPECT_EQ(cloud.header.frame_id, "map");
  EXPECT_EQ(cloud.header.stamp.sec, 10);
  EXPECT_EQ(cloud.header.stamp.nanosec, 200U);
}

TEST(GetPointCloud, EmptyMapProducesEmptyCloud) {
  OccupancyMap map(make_default_config());
  auto cloud = map.get_point_cloud("map", 0, 0U);
  EXPECT_EQ(cloud.width, 0U);
  EXPECT_TRUE(cloud.data.empty());
}

TEST(GetPointCloud, HasXYZFloat32Fields) {
  OccupancyMap map(make_default_config());
  auto input = make_cloud({{0.0F, 0.0F, 1.0F}});
  map.add_pointcloud(input);
  auto cloud = map.get_point_cloud("map", 0, 0U);

  ASSERT_EQ(cloud.fields.size(), 3u);
  EXPECT_EQ(cloud.fields[0].name, "x");
  EXPECT_EQ(cloud.fields[1].name, "y");
  EXPECT_EQ(cloud.fields[2].name, "z");
  for (const auto& f : cloud.fields) {
    EXPECT_EQ(f.datatype, sensor_msgs::msg::PointField::FLOAT32);
    EXPECT_EQ(f.count, 1U);
  }
}

TEST(GetPointCloud, PointStepIs12Bytes) {
  OccupancyMap map(make_default_config());
  auto input = make_cloud({{0.0F, 0.0F, 1.0F}});
  map.add_pointcloud(input);
  auto cloud = map.get_point_cloud("map", 0, 0U);
  EXPECT_EQ(cloud.point_step, 12U);
}

TEST(GetPointCloud, DataSizeMatchesDimensions) {
  OccupancyMap map(make_default_config());
  auto input = make_cloud({{0.0F, 0.0F, 1.0F}, {2.0F, 0.0F, 1.0F}});
  map.add_pointcloud(input);
  auto cloud = map.get_point_cloud("map", 0, 0U);
  EXPECT_EQ(cloud.data.size(), static_cast<size_t>(cloud.width) * cloud.point_step);
}

TEST(GetPointCloud, PointCountMatchesVoxelCount) {
  OccupancyMap map(make_default_config());
  // Three points in three distinct voxels
  auto input = make_cloud({
      {0.05F, 0.05F, 0.5F},
      {1.05F, 0.05F, 0.5F},
      {2.05F, 0.05F, 0.5F},
  });
  map.add_pointcloud(input);
  auto cloud = map.get_point_cloud("map", 0, 0U);
  EXPECT_EQ(static_cast<size_t>(cloud.width), map.voxel_count());
}

TEST(GetPointCloud, AllPointsAreFinite) {
  OccupancyMap map(make_default_config());
  auto input = make_cloud({
      {0.05F, 0.05F, 0.5F},
      {1.05F, 1.05F, 1.5F},
      {2.05F, 2.05F, 2.5F},
  });
  map.add_pointcloud(input);
  auto cloud = map.get_point_cloud("map", 0, 0U);

  for (uint32_t i = 0; i < cloud.width; ++i) {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    const size_t off = static_cast<size_t>(i) * cloud.point_step;
    std::memcpy(&x, &cloud.data[off], 4U);
    std::memcpy(&y, &cloud.data[off + 4U], 4U);
    std::memcpy(&z, &cloud.data[off + 8U], 4U);
    EXPECT_TRUE(std::isfinite(x)) << "Point " << i;
    EXPECT_TRUE(std::isfinite(y)) << "Point " << i;
    EXPECT_TRUE(std::isfinite(z)) << "Point " << i;
  }
}

TEST(GetPointCloud, PointCenterWithinVoxelBounds) {
  OccupancyMap map(make_default_config());
  // Insert a point at (0.05, 0.05, 0.5) → voxel (50,50,5) in a 100-voxel grid
  auto input = make_cloud({{0.05F, 0.05F, 0.5F}});
  map.add_pointcloud(input);
  auto cloud = map.get_point_cloud("map", 0, 0U);

  ASSERT_EQ(cloud.width, 1U);
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  std::memcpy(&x, &cloud.data[0U], 4U);
  std::memcpy(&y, &cloud.data[4U], 4U);
  std::memcpy(&z, &cloud.data[8U], 4U);

  // Voxel center should be within ±resolution/2 of the inserted point
  const float res = static_cast<float>(make_default_config().resolution);
  EXPECT_NEAR(x, 0.05F, res);
  EXPECT_NEAR(y, 0.05F, res);
  EXPECT_NEAR(z, 0.5F, res);
}

TEST(GetPointCloud, IsNotBigEndian) {
  OccupancyMap map(make_default_config());
  auto cloud = map.get_point_cloud("map", 0, 0U);
  EXPECT_FALSE(cloud.is_bigendian);
}

TEST(GetPointCloud, IsDense) {
  OccupancyMap map(make_default_config());
  auto input = make_cloud({{0.0F, 0.0F, 1.0F}});
  map.add_pointcloud(input);
  auto cloud = map.get_point_cloud("map", 0, 0U);
  EXPECT_TRUE(cloud.is_dense);
}

// ---------------------------------------------------------------------------
// voxel_count()
// ---------------------------------------------------------------------------

TEST(VoxelCount, ZeroAfterConstruction) {
  OccupancyMap map(make_default_config());
  EXPECT_EQ(map.voxel_count(), 0u);
}

TEST(VoxelCount, IncreasesWithDistinctVoxels) {
  OccupancyMap map(make_default_config());
  auto cloud = make_cloud({
      {0.05F, 0.05F, 0.5F},
      {1.05F, 0.05F, 0.5F},
      {2.05F, 0.05F, 0.5F},
  });
  map.add_pointcloud(cloud);
  EXPECT_EQ(map.voxel_count(), 3u);
}

TEST(VoxelCount, ZeroAfterClear) {
  OccupancyMap map(make_default_config());
  auto cloud = make_cloud({{0.0F, 0.0F, 1.0F}});
  map.add_pointcloud(cloud);
  map.clear();
  EXPECT_EQ(map.voxel_count(), 0u);
}

}  // namespace
}  // namespace slam_node
