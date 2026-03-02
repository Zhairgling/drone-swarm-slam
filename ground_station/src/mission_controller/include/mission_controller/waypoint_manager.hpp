#pragma once

#include <cstddef>
#include <vector>

namespace mission_controller {

/// A 3-D mission waypoint (map frame, metres).
struct Waypoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/// Pure-logic waypoint sequencer — no ROS dependency, fully testable.
///
/// Advances through an ordered list of 3-D waypoints. The current waypoint
/// is considered reached when the drone enters a configurable acceptance
/// radius sphere centred on the waypoint.
class WaypointManager {
 public:
  /// \param waypoints  Ordered mission waypoints. May be empty.
  /// \param radius_m   Acceptance radius in metres (should be > 0).
  WaypointManager(std::vector<Waypoint> waypoints, double radius_m);

  /// Update estimated drone position and advance the active waypoint if
  /// the drone is within the acceptance radius.
  ///
  /// \return true if the active waypoint changed (advanced or mission
  ///         completed), false otherwise.
  bool update_pose(double x, double y, double z);

  /// \return true if there is at least one waypoint still to visit.
  bool has_active_waypoint() const;

  /// Returns the current active waypoint.
  /// \pre has_active_waypoint() == true
  const Waypoint& current_waypoint() const;

  /// Zero-based index of the active waypoint in the original list.
  size_t current_index() const;

  /// Number of waypoints not yet visited, including the current one.
  size_t waypoints_remaining() const;

 private:
  std::vector<Waypoint> waypoints_;
  double radius_m_;
  size_t current_idx_ = 0;
};

}  // namespace mission_controller
