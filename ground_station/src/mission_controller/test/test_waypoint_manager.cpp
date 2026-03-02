// cppcheck-suppress-file syntaxError
#include "mission_controller/waypoint_manager.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace mission_controller {
namespace {

// --- empty waypoint list ---

TEST(WaypointManager, EmptyHasNoActiveWaypoint) {
  WaypointManager mgr({}, 1.0);
  EXPECT_FALSE(mgr.has_active_waypoint());
}

TEST(WaypointManager, EmptyRemainingIsZero) {
  WaypointManager mgr({}, 1.0);
  EXPECT_EQ(mgr.waypoints_remaining(), 0u);
}

TEST(WaypointManager, EmptyUpdatePoseReturnsFalse) {
  WaypointManager mgr({}, 1.0);
  EXPECT_FALSE(mgr.update_pose(0.0, 0.0, 0.0));
}

TEST(WaypointManager, EmptyCurrentIndexIsZero) {
  WaypointManager mgr({}, 1.0);
  EXPECT_EQ(mgr.current_index(), 0u);
}

// --- single waypoint ---

TEST(WaypointManager, SingleWaypointHasActive) {
  WaypointManager mgr({{1.0, 2.0, 3.0}}, 0.5);
  EXPECT_TRUE(mgr.has_active_waypoint());
}

TEST(WaypointManager, SingleWaypointRemainingIsOne) {
  WaypointManager mgr({{1.0, 2.0, 3.0}}, 0.5);
  EXPECT_EQ(mgr.waypoints_remaining(), 1u);
}

TEST(WaypointManager, CurrentWaypointMatchesFirst) {
  WaypointManager mgr({{1.0, 2.0, 3.0}}, 0.5);
  const auto& wp = mgr.current_waypoint();
  EXPECT_DOUBLE_EQ(wp.x, 1.0);
  EXPECT_DOUBLE_EQ(wp.y, 2.0);
  EXPECT_DOUBLE_EQ(wp.z, 3.0);
}

TEST(WaypointManager, CurrentIndexStartsAtZero) {
  WaypointManager mgr({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}, 0.5);
  EXPECT_EQ(mgr.current_index(), 0u);
}

// --- update_pose / advance logic ---

TEST(WaypointManager, FarAwayPoseDoesNotAdvance) {
  WaypointManager mgr({{0.0, 0.0, 0.0}}, 1.0);
  EXPECT_FALSE(mgr.update_pose(5.0, 5.0, 5.0));
  EXPECT_TRUE(mgr.has_active_waypoint());
}

TEST(WaypointManager, PoseAtWaypointAdvances) {
  WaypointManager mgr({{1.0, 2.0, 3.0}}, 0.5);
  EXPECT_TRUE(mgr.update_pose(1.0, 2.0, 3.0));
  EXPECT_FALSE(mgr.has_active_waypoint());
}

TEST(WaypointManager, PoseOnRadiusBoundaryAdvances) {
  // Distance from (1.5, 0, 0) to (1.0, 0, 0) is exactly 0.5
  WaypointManager mgr({{1.0, 0.0, 0.0}}, 0.5);
  EXPECT_TRUE(mgr.update_pose(1.5, 0.0, 0.0));
}

TEST(WaypointManager, PoseJustOutsideRadiusDoesNotAdvance) {
  // Distance from (1.6, 0, 0) to (1.0, 0, 0) is 0.6 > 0.5
  WaypointManager mgr({{1.0, 0.0, 0.0}}, 0.5);
  EXPECT_FALSE(mgr.update_pose(1.6, 0.0, 0.0));
  EXPECT_TRUE(mgr.has_active_waypoint());
}

TEST(WaypointManager, UpdatePoseReturnsTrueExactlyOnAdvance) {
  WaypointManager mgr({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}, 0.5);
  EXPECT_FALSE(mgr.update_pose(5.0, 5.0, 5.0));   // far
  EXPECT_TRUE(mgr.update_pose(0.0, 0.0, 0.0));    // reach wp 0
  EXPECT_FALSE(mgr.update_pose(5.0, 5.0, 5.0));   // far from wp 1
  EXPECT_TRUE(mgr.update_pose(1.0, 0.0, 0.0));    // reach wp 1
}

TEST(WaypointManager, CompletedMissionUpdateReturnsFalse) {
  WaypointManager mgr({{0.0, 0.0, 0.0}}, 0.5);
  mgr.update_pose(0.0, 0.0, 0.0);         // complete mission
  EXPECT_FALSE(mgr.update_pose(0.0, 0.0, 0.0));  // already done
}

// --- multi-waypoint sequence ---

TEST(WaypointManager, SequenceAdvancesInOrder) {
  std::vector<Waypoint> wps = {
      {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  WaypointManager mgr(wps, 0.5);

  EXPECT_EQ(mgr.current_index(), 0u);
  EXPECT_EQ(mgr.waypoints_remaining(), 3u);

  mgr.update_pose(0.0, 0.0, 0.0);
  EXPECT_EQ(mgr.current_index(), 1u);
  EXPECT_EQ(mgr.waypoints_remaining(), 2u);

  mgr.update_pose(1.0, 0.0, 0.0);
  EXPECT_EQ(mgr.current_index(), 2u);
  EXPECT_EQ(mgr.waypoints_remaining(), 1u);

  mgr.update_pose(2.0, 0.0, 0.0);
  EXPECT_FALSE(mgr.has_active_waypoint());
  EXPECT_EQ(mgr.waypoints_remaining(), 0u);
}

TEST(WaypointManager, CurrentWaypointUpdatesAfterAdvance) {
  WaypointManager mgr({{0.0, 0.0, 0.0}, {9.0, 9.0, 9.0}}, 0.5);
  mgr.update_pose(0.0, 0.0, 0.0);  // advance to wp 1
  const auto& wp = mgr.current_waypoint();
  EXPECT_DOUBLE_EQ(wp.x, 9.0);
  EXPECT_DOUBLE_EQ(wp.y, 9.0);
  EXPECT_DOUBLE_EQ(wp.z, 9.0);
}

TEST(WaypointManager, IntermediateWaypointNotSkipped) {
  // Drone jumps directly to last waypoint — intermediate waypoint should
  // still be advanced through if drone entered its radius first.
  WaypointManager mgr({{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}}, 0.5);
  // Drone at (5, 0, 0) is far from wp 0 — should NOT skip to wp 1.
  mgr.update_pose(5.0, 0.0, 0.0);
  EXPECT_EQ(mgr.current_index(), 0u);
}

TEST(WaypointManager, ThreeDimensionalDistanceUsed) {
  // Waypoint at (0,0,1). Drone at (0,0,0) — distance = 1.0 > radius 0.5.
  WaypointManager mgr({{0.0, 0.0, 1.0}}, 0.5);
  EXPECT_FALSE(mgr.update_pose(0.0, 0.0, 0.0));
  // Move to (0, 0, 0.6) — distance = 0.4 <= 0.5.
  EXPECT_TRUE(mgr.update_pose(0.0, 0.0, 0.6));
}

}  // namespace
}  // namespace mission_controller
