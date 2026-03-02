#include "mission_controller/waypoint_manager.hpp"

#include <cmath>

namespace mission_controller {

WaypointManager::WaypointManager(std::vector<Waypoint> waypoints,
                                 double radius_m)
    : waypoints_(std::move(waypoints)), radius_m_(radius_m) {}

bool WaypointManager::update_pose(double x, double y, double z) {
  if (!has_active_waypoint()) {
    return false;
  }

  const auto& wp = waypoints_[current_idx_];
  double dx = x - wp.x;
  double dy = y - wp.y;
  double dz = z - wp.z;
  double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  if (dist <= radius_m_) {
    ++current_idx_;
    return true;
  }
  return false;
}

bool WaypointManager::has_active_waypoint() const {
  return current_idx_ < waypoints_.size();
}

const Waypoint& WaypointManager::current_waypoint() const {
  return waypoints_[current_idx_];
}

size_t WaypointManager::current_index() const { return current_idx_; }

size_t WaypointManager::waypoints_remaining() const {
  if (current_idx_ >= waypoints_.size()) {
    return 0;
  }
  return waypoints_.size() - current_idx_;
}

}  // namespace mission_controller
