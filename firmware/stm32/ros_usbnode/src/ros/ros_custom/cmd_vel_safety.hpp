#pragma once

#include <cmath>
#include <cstdint>

namespace mowgli_cmd_vel
{

struct SafetyState
{
  float cmd_wz;
  float left_target_mps;
  float right_target_mps;
  uint32_t last_valid_tick;
  bool zero_motion_intent = false;
  bool yaw_inhibited = true;
};

/// Accept only finite wire values; invalid input clears targets but preserves
/// the last-valid timestamp so it cannot refresh the motion watchdog.
inline bool apply_safety(float vx, float wz, uint32_t tick, SafetyState &state)
{
  if (!std::isfinite(vx) || !std::isfinite(wz)) {
    state.cmd_wz = 0.0f;
    state.left_target_mps = 0.0f;
    state.right_target_mps = 0.0f;
    state.zero_motion_intent = false;
    state.yaw_inhibited = true;
    return false;
  }
  state.cmd_wz = wz;
  state.last_valid_tick = tick;
  state.zero_motion_intent = vx == 0.0f && wz == 0.0f;
  state.yaw_inhibited = state.zero_motion_intent;
  return true;
}

/// Ignore CMD_VEL while the host explicitly places the robot in IDLE. In
/// particular, do not let an IDLE packet clear the yaw/motor stop gates before
/// a later MOWING state; a fresh post-IDLE command must do that.
inline bool apply_safety_for_mode(float vx, float wz, uint32_t tick,
                                  bool idle, SafetyState &state)
{
  if (idle) {
    state.cmd_wz = 0.0f;
    state.left_target_mps = 0.0f;
    state.right_target_mps = 0.0f;
    state.zero_motion_intent = true;
    state.yaw_inhibited = true;
    return false;
  }
  return apply_safety(vx, wz, tick, state);
}

}  // namespace mowgli_cmd_vel
