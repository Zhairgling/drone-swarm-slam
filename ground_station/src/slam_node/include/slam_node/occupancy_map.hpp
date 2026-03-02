#pragma once

#include <cstdint>
#include <string>
#include <unordered_set>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace slam_node {

struct MapConfig {
  double resolution = 0.1;   // meters per voxel edge
  double size_x = 20.0;      // total map width in X (meters), centered at 0
  double size_y = 20.0;      // total map height in Y (meters), centered at 0
  double height_min = -0.5;  // minimum Z to include (meters)
  double height_max = 3.0;   // maximum Z to include (meters)
};

/// 3D voxel occupancy map with 2D OccupancyGrid projection.
///
/// Accumulates PointCloud2 frames into a fixed-resolution voxel grid.
/// Provides 2D occupancy grid (XY column projection) and 3D output.
///
/// DESIGN: Voxel keys are packed int64 values (linear index into 3D grid).
/// This avoids the overhead of a map with struct keys and custom hash.
///
/// NOT thread-safe — caller must synchronize if using from multiple threads.
class OccupancyMap {
 public:
  explicit OccupancyMap(const MapConfig& config);

  /// Add points from a PointCloud2 message (x, y, z float32 fields).
  /// Points outside map bounds or with non-finite coordinates are ignored.
  void add_pointcloud(const sensor_msgs::msg::PointCloud2& cloud);

  /// Build a 2D OccupancyGrid from the current voxel map.
  /// A cell is 100 (occupied) if any voxel in its Z column is filled.
  /// Cells with no data remain -1 (unknown).
  nav_msgs::msg::OccupancyGrid get_occupancy_grid(const std::string& frame_id,
                                                   int32_t stamp_sec,
                                                   uint32_t stamp_nanosec) const;

  /// Return all occupied voxel centers as a PointCloud2.
  sensor_msgs::msg::PointCloud2 get_point_cloud(const std::string& frame_id,
                                                 int32_t stamp_sec,
                                                 uint32_t stamp_nanosec) const;

  /// Number of currently occupied voxels.
  size_t voxel_count() const { return voxels_.size(); }

  /// Remove all occupied voxels.
  void clear() { voxels_.clear(); }

  const MapConfig& config() const { return config_; }

 private:
  // Grid dimensions (clamped to minimum 1 to prevent division by zero).
  int grid_nx() const;
  int grid_ny() const;
  int grid_nz() const;

  // Pack voxel index (ix, iy, iz) into a single int64 key.
  int64_t voxel_key(int ix, int iy, int iz) const;

  MapConfig config_;
  std::unordered_set<int64_t> voxels_;
};

}  // namespace slam_node
