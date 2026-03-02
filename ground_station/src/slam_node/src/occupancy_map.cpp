#include "slam_node/occupancy_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <sensor_msgs/msg/point_field.hpp>

namespace slam_node {

OccupancyMap::OccupancyMap(const MapConfig& config) : config_(config) {}

int OccupancyMap::grid_nx() const {
  return std::max(1, static_cast<int>(std::ceil(config_.size_x / config_.resolution)));
}

int OccupancyMap::grid_ny() const {
  return std::max(1, static_cast<int>(std::ceil(config_.size_y / config_.resolution)));
}

int OccupancyMap::grid_nz() const {
  double z_range = config_.height_max - config_.height_min;
  return std::max(1, static_cast<int>(std::ceil(z_range / config_.resolution)));
}

int64_t OccupancyMap::voxel_key(int ix, int iy, int iz) const {
  // DESIGN: Linear 3D index: key = ix*(ny*nz) + iy*nz + iz.
  // Max value for default config (200x200x35): ~1.4M — fits in int32,
  // but int64 gives headroom for larger maps without overflow risk.
  return static_cast<int64_t>(ix) * grid_ny() * grid_nz() +
         static_cast<int64_t>(iy) * grid_nz() +
         static_cast<int64_t>(iz);
}

void OccupancyMap::add_pointcloud(const sensor_msgs::msg::PointCloud2& cloud) {
  if (cloud.data.empty() || cloud.point_step == 0) {
    return;
  }

  // Locate x, y, z float32 field offsets (defaults match PointCloud2 convention).
  uint32_t x_off = 0;
  uint32_t y_off = 4;
  uint32_t z_off = 8;
  bool found_x = false;
  bool found_y = false;
  bool found_z = false;

  for (const auto& field : cloud.fields) {
    if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count < 1) {
      continue;
    }
    if (field.name == "x") { x_off = field.offset; found_x = true; }
    else if (field.name == "y") { y_off = field.offset; found_y = true; }
    else if (field.name == "z") { z_off = field.offset; found_z = true; }
  }

  if (!found_x || !found_y || !found_z) {
    return;
  }

  const double half_x = config_.size_x / 2.0;
  const double half_y = config_.size_y / 2.0;
  const int nx = grid_nx();
  const int ny = grid_ny();
  const int nz = grid_nz();

  for (uint32_t row = 0; row < cloud.height; ++row) {
    for (uint32_t col = 0; col < cloud.width; ++col) {
      const size_t base =
          static_cast<size_t>(row) * cloud.row_step +
          static_cast<size_t>(col) * cloud.point_step;

      // Guard against reads past the end of the buffer for any field.
      constexpr size_t kF32 = 4U;
      const uint32_t max_off = std::max({x_off, y_off, z_off});
      if (base + static_cast<size_t>(max_off) + kF32 > cloud.data.size()) {
        continue;
      }

      float xf = 0.0F;
      float yf = 0.0F;
      float zf = 0.0F;
      std::memcpy(&xf, &cloud.data[base + x_off], kF32);
      std::memcpy(&yf, &cloud.data[base + y_off], kF32);
      std::memcpy(&zf, &cloud.data[base + z_off], kF32);

      const double x = xf;
      const double y = yf;
      const double z = zf;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      if (x < -half_x || x >= half_x ||
          y < -half_y || y >= half_y ||
          z < config_.height_min || z >= config_.height_max) {
        continue;
      }

      const int ix = std::clamp(
          static_cast<int>((x + half_x) / config_.resolution), 0, nx - 1);
      const int iy = std::clamp(
          static_cast<int>((y + half_y) / config_.resolution), 0, ny - 1);
      const int iz = std::clamp(
          static_cast<int>((z - config_.height_min) / config_.resolution), 0, nz - 1);

      voxels_.insert(voxel_key(ix, iy, iz));
    }
  }
}

nav_msgs::msg::OccupancyGrid OccupancyMap::get_occupancy_grid(
    const std::string& frame_id, int32_t stamp_sec,
    uint32_t stamp_nanosec) const {
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = frame_id;
  grid.header.stamp.sec = stamp_sec;
  grid.header.stamp.nanosec = stamp_nanosec;

  const int nx = grid_nx();
  const int ny = grid_ny();
  const int nz = grid_nz();

  grid.info.resolution = static_cast<float>(config_.resolution);
  grid.info.width = static_cast<uint32_t>(nx);
  grid.info.height = static_cast<uint32_t>(ny);
  grid.info.origin.position.x = -config_.size_x / 2.0;
  grid.info.origin.position.y = -config_.size_y / 2.0;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.info.map_load_time = grid.header.stamp;

  // Initialize all cells to unknown (-1 in ROS OccupancyGrid convention).
  grid.data.assign(static_cast<size_t>(nx) * static_cast<size_t>(ny),
                   static_cast<int8_t>(-1));

  // Project occupied voxels onto the XY plane.
  // Reverse key: key = ix*(ny*nz) + iy*nz + iz
  for (const int64_t key : voxels_) {
    const int iz = static_cast<int>(key % nz);
    const int64_t rem = key / nz;
    const int iy = static_cast<int>(rem % ny);
    const int ix = static_cast<int>(rem / ny);

    // Bounds check: all keys were inserted via voxel_key with valid indices,
    // but guard against grid dimension mismatch (defensive).
    if (ix < 0 || ix >= nx || iy < 0 || iy >= ny || iz < 0 || iz >= nz) {
      continue;
    }

    // OccupancyGrid is row-major (row = iy, col = ix).
    grid.data[static_cast<size_t>(iy) * static_cast<size_t>(nx) +
              static_cast<size_t>(ix)] = static_cast<int8_t>(100);
  }

  return grid;
}

sensor_msgs::msg::PointCloud2 OccupancyMap::get_point_cloud(
    const std::string& frame_id, int32_t stamp_sec,
    uint32_t stamp_nanosec) const {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp.sec = stamp_sec;
  cloud.header.stamp.nanosec = stamp_nanosec;

  constexpr uint32_t kFloat32Size = 4U;
  constexpr uint32_t kPointStep = kFloat32Size * 3U;

  sensor_msgs::msg::PointField fx;
  fx.name = "x";
  fx.offset = 0U;
  fx.datatype = sensor_msgs::msg::PointField::FLOAT32;
  fx.count = 1U;

  sensor_msgs::msg::PointField fy;
  fy.name = "y";
  fy.offset = kFloat32Size;
  fy.datatype = sensor_msgs::msg::PointField::FLOAT32;
  fy.count = 1U;

  sensor_msgs::msg::PointField fz;
  fz.name = "z";
  fz.offset = kFloat32Size * 2U;
  fz.datatype = sensor_msgs::msg::PointField::FLOAT32;
  fz.count = 1U;

  cloud.fields = {fx, fy, fz};
  cloud.is_bigendian = false;
  cloud.point_step = kPointStep;
  cloud.height = 1U;
  cloud.is_dense = true;

  const int nx = grid_nx();
  const int ny = grid_ny();
  const int nz = grid_nz();
  const double half_x = config_.size_x / 2.0;
  const double half_y = config_.size_y / 2.0;

  // Pre-allocate for all voxels; resize down if any are skipped.
  const size_t n = voxels_.size();
  cloud.data.resize(n * static_cast<size_t>(kPointStep));
  uint32_t point_count = 0U;

  for (const int64_t key : voxels_) {
    const int iz = static_cast<int>(key % nz);
    const int64_t rem = key / nz;
    const int iy = static_cast<int>(rem % ny);
    const int ix = static_cast<int>(rem / ny);

    if (ix < 0 || ix >= nx || iy < 0 || iy >= ny || iz < 0 || iz >= nz) {
      continue;
    }

    // Voxel center coordinates.
    const auto xf = static_cast<float>(
        -half_x + (static_cast<double>(ix) + 0.5) * config_.resolution);
    const auto yf = static_cast<float>(
        -half_y + (static_cast<double>(iy) + 0.5) * config_.resolution);
    const auto zf = static_cast<float>(
        config_.height_min + (static_cast<double>(iz) + 0.5) * config_.resolution);

    uint8_t* ptr = &cloud.data[static_cast<size_t>(point_count) * kPointStep];
    std::memcpy(ptr, &xf, kFloat32Size);
    std::memcpy(ptr + kFloat32Size, &yf, kFloat32Size);
    std::memcpy(ptr + kFloat32Size * 2U, &zf, kFloat32Size);
    ++point_count;
  }

  cloud.width = point_count;
  cloud.row_step = cloud.width * kPointStep;
  cloud.data.resize(static_cast<size_t>(point_count) * kPointStep);

  return cloud;
}

}  // namespace slam_node
