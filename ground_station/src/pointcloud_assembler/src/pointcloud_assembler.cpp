#include "pointcloud_assembler/pointcloud_assembler.hpp"

#include <cstring>
#include <numeric>

namespace pointcloud_assembler {

PointcloudAssembler::PointcloudAssembler(const AssemblerConfig& config)
    : config_(config) {}

void PointcloudAssembler::add_cloud(
    const sensor_msgs::msg::PointCloud2& cloud) {
  // DESIGN: compute stamp as a single int64_t for easy window arithmetic.
  int64_t stamp_ns = static_cast<int64_t>(cloud.header.stamp.sec) *
                         1000000000LL +
                     static_cast<int64_t>(cloud.header.stamp.nanosec);

  // Evict oldest entry if at capacity.
  while (buffer_.size() >= config_.max_clouds) {
    buffer_.pop_front();
  }

  buffer_.push_back({cloud, stamp_ns});
}

void PointcloudAssembler::expire_old_clouds(int64_t reference_ns) {
  int64_t window_ns =
      static_cast<int64_t>(config_.window_duration_sec * 1.0e9);
  int64_t cutoff_ns = reference_ns - window_ns;

  while (!buffer_.empty() && buffer_.front().stamp_ns < cutoff_ns) {
    buffer_.pop_front();
  }
}

sensor_msgs::msg::PointCloud2 PointcloudAssembler::assemble(
    const std::string& frame_id, int32_t stamp_sec,
    uint32_t stamp_nanosec) const {
  sensor_msgs::msg::PointCloud2 out;
  out.header.frame_id = frame_id;
  out.header.stamp.sec = stamp_sec;
  out.header.stamp.nanosec = stamp_nanosec;
  out.height = 1;
  out.is_dense = true;
  out.is_bigendian = false;

  if (buffer_.empty()) {
    out.width = 0;
    out.point_step = 0;
    out.row_step = 0;
    return out;
  }

  // Use the field layout from the first cloud.
  const auto& first = buffer_.front().cloud;
  out.fields = first.fields;
  out.point_step = first.point_step;
  out.is_bigendian = first.is_bigendian;

  if (out.point_step == 0) {
    out.width = 0;
    out.row_step = 0;
    return out;
  }

  // Sum data bytes from all clouds with a matching point_step.
  size_t total_bytes = std::accumulate(
      buffer_.begin(), buffer_.end(), size_t{0},
      [&out](size_t acc, const auto& entry) {
        return acc + (entry.cloud.point_step == out.point_step
                          ? entry.cloud.data.size()
                          : size_t{0});
      });

  out.data.resize(total_bytes);
  size_t offset = 0;
  for (const auto& entry : buffer_) {
    if (entry.cloud.point_step == out.point_step) {
      std::memcpy(out.data.data() + offset, entry.cloud.data.data(),
                  entry.cloud.data.size());
      offset += entry.cloud.data.size();
    }
  }

  out.width = static_cast<uint32_t>(total_bytes / out.point_step);
  out.row_step = out.width * out.point_step;

  return out;
}

size_t PointcloudAssembler::buffer_size() const { return buffer_.size(); }

size_t PointcloudAssembler::total_points() const {
  return std::accumulate(
      buffer_.begin(), buffer_.end(), size_t{0},
      [](size_t acc, const auto& entry) {
        return acc + (entry.cloud.point_step > 0
                          ? entry.cloud.data.size() / entry.cloud.point_step
                          : size_t{0});
      });
}

}  // namespace pointcloud_assembler
