#include <limits>

#include <unity.h>

#include "cmd_vel_safety.hpp"
#include "imu/imu_mount_transform.h"

using mowgli_cmd_vel::SafetyState;
using mowgli_cmd_vel::apply_safety;

static void test_finite_and_zero_are_accepted()
{
  SafetyState state{3.0f, 1.0f, -1.0f, 10u};
  TEST_ASSERT_TRUE(apply_safety(0.25f, -0.8f, 20u, state));
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -0.8f, state.cmd_wz);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 1.0f, state.left_target_mps);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, state.right_target_mps);
  TEST_ASSERT_EQUAL_UINT32(20u, state.last_valid_tick);
  TEST_ASSERT_TRUE(apply_safety(0.0f, 0.0f, 30u, state));
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.cmd_wz);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 1.0f, state.left_target_mps);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, state.right_target_mps);
  TEST_ASSERT_EQUAL_UINT32(30u, state.last_valid_tick);
}

static void test_nonfinite_commands_clear_targets_without_refresh()
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const float values[] = {nan, inf, -inf};
  for (float vx : values) {
    SafetyState state{0.7f, 0.4f, -0.4f, 77u};
    TEST_ASSERT_FALSE(apply_safety(vx, 0.0f, 99u, state));
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.cmd_wz);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.left_target_mps);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.right_target_mps);
    TEST_ASSERT_EQUAL_UINT32(77u, state.last_valid_tick);
  }
  for (float wz : values) {
    SafetyState state{0.7f, 0.4f, -0.4f, 77u};
    TEST_ASSERT_FALSE(apply_safety(0.0f, wz, 99u, state));
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.cmd_wz);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.left_target_mps);
    TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.right_target_mps);
    TEST_ASSERT_EQUAL_UINT32(77u, state.last_valid_tick);
  }
  for (float vx : values) {
    for (float wz : values) {
      SafetyState state{0.7f, 0.4f, -0.4f, 77u};
      TEST_ASSERT_FALSE(apply_safety(vx, wz, 99u, state));
      TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.cmd_wz);
      TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.left_target_mps);
      TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, state.right_target_mps);
      TEST_ASSERT_EQUAL_UINT32(77u, state.last_valid_tick);
    }
  }
}

static void test_valid_command_after_invalid_is_normal()
{
  SafetyState state{0.0f, 0.0f, 0.0f, 11u};
  TEST_ASSERT_FALSE(apply_safety(
      std::numeric_limits<float>::quiet_NaN(), 0.0f, 12u, state));
  TEST_ASSERT_TRUE(apply_safety(0.3f, -0.2f, 13u, state));
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -0.2f, state.cmd_wz);
  TEST_ASSERT_EQUAL_UINT32(13u, state.last_valid_tick);
}

static void test_imu_mount_rotation_identity_preserves_all_axes()
{
  float x = 1.0f;
  float y = -2.0f;
  float z = 3.0f;
  IMU_ApplyMountRotation(IMU_MOUNT_ROTATION_IDENTITY, &x, &y, &z);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 1.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -2.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 3.0f, z);
}

static void test_imu_mount_rotation_yaw_180_negates_x_y_only()
{
  float x = 1.0f;
  float y = -2.0f;
  float z = 3.0f;
  IMU_ApplyMountRotation(IMU_MOUNT_ROTATION_YAW_180, &x, &y, &z);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 2.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 3.0f, z);
}

static void test_configured_mount_maps_accel_and_gyro_basis_vectors()
{
#if IMU_MOUNT_ROTATION == IMU_MOUNT_ROTATION_YAW_180
  float x = 1.0f, y = 0.0f, z = 0.0f;
  IMU_ApplyConfiguredMountRotation(&x, &y, &z);  // +X acceleration
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, z);

  x = 0.0f; y = 1.0f; z = 0.0f;
  IMU_ApplyConfiguredMountRotation(&x, &y, &z);  // +Y acceleration
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, z);

  x = 1.0f; y = 0.0f; z = 0.0f;
  IMU_ApplyConfiguredMountRotation(&x, &y, &z);  // gyro +X
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, z);

  x = 0.0f; y = 1.0f; z = 0.0f;
  IMU_ApplyConfiguredMountRotation(&x, &y, &z);  // gyro +Y
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, z);

  x = 0.0f; y = 0.0f; z = 1.0f;
  IMU_ApplyConfiguredMountRotation(&x, &y, &z);  // gyro +Z
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 0.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 1.0f, z);

  x = 1.0f; y = 2.0f; z = 3.0f;
  IMU_ApplyConfiguredMountRotation(&x, &y, &z);  // magnetometer path
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -1.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, -2.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 3.0f, z);
#else
  float x = 1.0f, y = 2.0f, z = 3.0f;
  IMU_ApplyConfiguredMountRotation(&x, &y, &z);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 1.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 2.0f, y);
  TEST_ASSERT_FLOAT_WITHIN(0.0f, 3.0f, z);
#endif
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_finite_and_zero_are_accepted);
  RUN_TEST(test_nonfinite_commands_clear_targets_without_refresh);
  RUN_TEST(test_valid_command_after_invalid_is_normal);
  RUN_TEST(test_imu_mount_rotation_identity_preserves_all_axes);
  RUN_TEST(test_imu_mount_rotation_yaw_180_negates_x_y_only);
  RUN_TEST(test_configured_mount_maps_accel_and_gyro_basis_vectors);
  return UNITY_END();
}
