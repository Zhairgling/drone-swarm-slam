#pragma once

#include <cstdint>
#include <deque>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pointcloud_assembler {

struct AssemblerConfig {
  /// Time window: clouds older than this relative to the reference are expired.
  double window_duration_sec = 1.0;
  /// Maximum number of clouds to hold in the buffer at any time.
  size_t max_clouds = 100;
};

/// Buffers incoming PointCloud2 messages and concatenates them into one cloud.
///
/// Separated from the ROS node for testability. All methods are pure logic
/// with no ROS dependencies except the message type.
///
/// DESIGN: This is a simple time-windowed accumulator. It does not perform
/// coordinate-frame transforms — callers (i.e. PointcloudAssemblerNode) are
/// responsible for transforming all clouds to a common frame via TF2 before
/// calling add_cloud().
class PointcloudAssembler {
 public:
  explicit PointcloudAssembler(const AssemblerConfig& config);

  /// Add a cloud to the buffer.
  /// If the buffer is at max_clouds capacity, the oldest cloud is evicted.
  void add_cloud(const sensor_msgs::msg::PointCloud2& cloud);

  /// Remove clouds whose timestamp is older than (reference_ns - window_ns).
  /// reference_ns is typically the current ROS time in nanoseconds.
  void expire_old_clouds(int64_t reference_ns);

  /// Concatenate all buffered clouds into one PointCloud2.
  /// Returns an empty cloud if the buffer is empty.
  /// Only clouds whose point_step matches the first cloud's are included.
  sensor_msgs::msg::PointCloud2 assemble(const std::string& frame_id,
                                         int32_t stamp_sec,
                                         uint32_t stamp_nanosec) const;

  /// Number of clouds currently in the buffer.
  size_t buffer_size() const;

  /// Total number of points across all buffered clouds.
  size_t total_points() const;

  const AssemblerConfig& config() const { return config_; }

 private:
  struct CloudEntry {
    sensor_msgs::msg::PointCloud2 cloud;
    // cppcheck-suppress unusedStructMember
    int64_t stamp_ns;  ///< stamp in nanoseconds since epoch for expiry checks
  };

  AssemblerConfig config_;
  std::deque<CloudEntry> buffer_;
};

}  // namespace pointcloud_assembler
