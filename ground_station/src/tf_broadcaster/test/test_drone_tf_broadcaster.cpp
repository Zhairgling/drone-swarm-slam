// cppcheck-suppress-file syntaxError
#include "tf_broadcaster/drone_tf_broadcaster.hpp"

#include <cmath>

#include <gtest/gtest.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace tf_broadcaster {
namespace {

// --- default_config ---

TEST(DefaultConfig, FrontSensorFacesPositiveX) {
  auto cfg = default_config();
  EXPECT_NEAR(cfg.tof_front.x, kDefaultSensorOffset, 1e-9);
  EXPECT_NEAR(cfg.tof_front.y, 0.0, 1e-9);
  EXPECT_NEAR(cfg.tof_front.yaw, 0.0, 1e-9);
}

TEST(DefaultConfig, RightSensorFacesNegativeY) {
  auto cfg = default_config();
  EXPECT_NEAR(cfg.tof_right.x, 0.0, 1e-9);
  EXPECT_NEAR(cfg.tof_right.y, -kDefaultSensorOffset, 1e-9);
  EXPECT_NEAR(cfg.tof_right.yaw, -M_PI / 2.0, 1e-9);
}

TEST(DefaultConfig, BackSensorFacesNegativeX) {
  auto cfg = default_config();
  EXPECT_NEAR(cfg.tof_back.x, -kDefaultSensorOffset, 1e-9);
  EXPECT_NEAR(cfg.tof_back.y, 0.0, 1e-9);
  EXPECT_NEAR(cfg.tof_back.yaw, M_PI, 1e-9);
}

TEST(DefaultConfig, LeftSensorFacesPositiveY) {
  auto cfg = default_config();
  EXPECT_NEAR(cfg.tof_left.x, 0.0, 1e-9);
  EXPECT_NEAR(cfg.tof_left.y, kDefaultSensorOffset, 1e-9);
  EXPECT_NEAR(cfg.tof_left.yaw, M_PI / 2.0, 1e-9);
}

TEST(DefaultConfig, CameraAtOriginNoRotation) {
  auto cfg = default_config();
  EXPECT_NEAR(cfg.camera.x, 0.0, 1e-9);
  EXPECT_NEAR(cfg.camera.y, 0.0, 1e-9);
  EXPECT_NEAR(cfg.camera.z, 0.0, 1e-9);
  EXPECT_NEAR(cfg.camera.yaw, 0.0, 1e-9);
}

TEST(DefaultConfig, DefaultDroneIdIsOne) {
  auto cfg = default_config();
  EXPECT_EQ(cfg.drone_id, 1);
}

// --- make_transform ---

TEST(MakeTransform, FrameNamesAreSet) {
  SensorOffset offset{};
  auto t = make_transform("parent/base_link", "parent/sensor", offset);
  EXPECT_EQ(t.header.frame_id, "parent/base_link");
  EXPECT_EQ(t.child_frame_id, "parent/sensor");
}

TEST(MakeTransform, ZeroOffsetProducesIdentityRotation) {
  SensorOffset offset{0.0, 0.0, 0.0, 0.0};
  auto t = make_transform("p", "c", offset);
  EXPECT_NEAR(t.transform.rotation.x, 0.0, 1e-9);
  EXPECT_NEAR(t.transform.rotation.y, 0.0, 1e-9);
  EXPECT_NEAR(t.transform.rotation.z, 0.0, 1e-9);
  EXPECT_NEAR(t.transform.rotation.w, 1.0, 1e-9);
}

TEST(MakeTransform, TranslationMatchesOffset) {
  SensorOffset offset{0.05, -0.05, 0.01, 0.0};
  auto t = make_transform("p", "c", offset);
  EXPECT_NEAR(t.transform.translation.x, 0.05, 1e-9);
  EXPECT_NEAR(t.transform.translation.y, -0.05, 1e-9);
  EXPECT_NEAR(t.transform.translation.z, 0.01, 1e-9);
}

TEST(MakeTransform, YawPiHalfProducesCorrectQuaternion) {
  SensorOffset offset{0.0, 0.0, 0.0, M_PI / 2.0};
  auto t = make_transform("p", "c", offset);
  EXPECT_NEAR(t.transform.rotation.x, 0.0, 1e-9);
  EXPECT_NEAR(t.transform.rotation.y, 0.0, 1e-9);
  EXPECT_NEAR(t.transform.rotation.z, std::sin(M_PI / 4.0), 1e-9);
  EXPECT_NEAR(t.transform.rotation.w, std::cos(M_PI / 4.0), 1e-9);
}

TEST(MakeTransform, YawNegPiHalfProducesCorrectQuaternion) {
  SensorOffset offset{0.0, 0.0, 0.0, -M_PI / 2.0};
  auto t = make_transform("p", "c", offset);
  EXPECT_NEAR(t.transform.rotation.z, std::sin(-M_PI / 4.0), 1e-9);
  EXPECT_NEAR(t.transform.rotation.w, std::cos(-M_PI / 4.0), 1e-9);
}

TEST(MakeTransform, YawPiProducesCorrectQuaternion) {
  SensorOffset offset{0.0, 0.0, 0.0, M_PI};
  auto t = make_transform("p", "c", offset);
  // q = [0, 0, sin(pi/2), cos(pi/2)] = [0, 0, 1, 0]
  EXPECT_NEAR(t.transform.rotation.z, 1.0, 1e-9);
  EXPECT_NEAR(t.transform.rotation.w, 0.0, 1e-9);
}

TEST(MakeTransform, UsesZeroTimestamp) {
  SensorOffset offset{};
  auto t = make_transform("p", "c", offset);
  EXPECT_EQ(t.header.stamp.sec, 0);
  EXPECT_EQ(t.header.stamp.nanosec, 0u);
}

// --- build_transforms ---

TEST(BuildTransforms, ReturnsFiveTransforms) {
  auto transforms = build_transforms(default_config());
  EXPECT_EQ(transforms.size(), 5u);
}

TEST(BuildTransforms, ParentFrameIsBaseLink) {
  auto transforms = build_transforms(default_config());
  for (const auto& t : transforms) {
    EXPECT_EQ(t.header.frame_id, "drone_1/base_link");
  }
}

TEST(BuildTransforms, ChildFrameNamesUseDroneIdNamespace) {
  auto transforms = build_transforms(default_config());
  EXPECT_EQ(transforms[0].child_frame_id, "drone_1/tof_front");
  EXPECT_EQ(transforms[1].child_frame_id, "drone_1/tof_right");
  EXPECT_EQ(transforms[2].child_frame_id, "drone_1/tof_back");
  EXPECT_EQ(transforms[3].child_frame_id, "drone_1/tof_left");
  EXPECT_EQ(transforms[4].child_frame_id, "drone_1/camera");
}

TEST(BuildTransforms, DroneIdTwoUsesCorrectNamespace) {
  TfConfig cfg = default_config();
  cfg.drone_id = 2;
  auto transforms = build_transforms(cfg);
  EXPECT_EQ(transforms[0].header.frame_id, "drone_2/base_link");
  EXPECT_EQ(transforms[0].child_frame_id, "drone_2/tof_front");
}

TEST(BuildTransforms, FrontTransformHasCorrectTranslation) {
  auto transforms = build_transforms(default_config());
  const auto& t = transforms[0];  // tof_front
  EXPECT_NEAR(t.transform.translation.x, kDefaultSensorOffset, 1e-9);
  EXPECT_NEAR(t.transform.translation.y, 0.0, 1e-9);
}

TEST(BuildTransforms, RightTransformHasCorrectTranslation) {
  auto transforms = build_transforms(default_config());
  const auto& t = transforms[1];  // tof_right
  EXPECT_NEAR(t.transform.translation.y, -kDefaultSensorOffset, 1e-9);
}

}  // namespace
}  // namespace tf_broadcaster
