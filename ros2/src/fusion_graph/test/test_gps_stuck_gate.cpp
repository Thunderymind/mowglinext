// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the GPS payload-value staleness gate. Pure logic, no
// ROS/GTSAM. See gps_stuck_gate.hpp for the field evidence this guards
// against (mowglinext#694/#695: /gps/fix frozen bit-exact for entire
// sessions under a genuinely advancing receipt stamp and a receiver that
// never reports DEAD_RECKONING).

#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>

#include "fusion_graph/gps_stuck_gate.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

// ── GpsStuckImplausible ──────────────────────────────────────────────────

TEST(GpsStuckImplausible, NoWheelTravelSinceValueChangedIsPlausible)
{
  // Value just changed (or robot hasn't moved) — nothing to flag yet.
  EXPECT_FALSE(fg::GpsStuckImplausible(/*wheel_dist=*/0.0,
                                       /*abs_dtheta=*/0.0,
                                       /*min_wheel_dist=*/1.0,
                                       /*max_yaw=*/1.047));
}

TEST(GpsStuckImplausible, WithinThresholdIsPlausible)
{
  EXPECT_FALSE(fg::GpsStuckImplausible(0.5, 0.0, 1.0, 1.047));
}

TEST(GpsStuckImplausible, ExactlyAtThresholdIsNotImplausible)
{
  // Strict > only, matching GpsJumpImplausible's convention.
  EXPECT_FALSE(fg::GpsStuckImplausible(1.0, 0.0, 1.0, 1.047));
}

TEST(GpsStuckImplausible, JustOverThresholdIsImplausible)
{
  EXPECT_TRUE(fg::GpsStuckImplausible(1.000001, 0.0, 1.0, 1.047));
}

TEST(GpsStuckImplausible, FieldScenario_TwoSessionsBothTrip)
{
  // gps-freeze-investigation-20260920-1836: 4.5 m wheel travel, value frozen
  // the entire ~70 s session.
  EXPECT_TRUE(fg::GpsStuckImplausible(4.5, 0.0, 1.0, 1.047));
  // gps-freeze-investigation-20260920-1911: 2.66 m wheel travel, value
  // frozen the entire ~50 s session.
  EXPECT_TRUE(fg::GpsStuckImplausible(2.66, 0.0, 1.0, 1.047));
}

TEST(GpsStuckImplausible, StandsDownMidTurn)
{
  // Large accumulated rotation since the value last changed: net
  // translation is not a reliable signal mid-turn (Invariant 16 reasoning),
  // even with plenty of wheel travel piled up.
  EXPECT_FALSE(fg::GpsStuckImplausible(/*wheel_dist=*/5.0,
                                       /*abs_dtheta=*/M_PI,
                                       /*min_wheel_dist=*/1.0,
                                       /*max_yaw=*/1.047));
}

TEST(GpsStuckImplausible, SmallRotationStaysWithinStandDownBudget)
{
  // A little rotation (e.g. steering wobble on a straight swath) must not
  // disable the check entirely.
  EXPECT_TRUE(fg::GpsStuckImplausible(2.0, 0.1, 1.0, 1.047));
}

// ── GpsValueChanged ───────────────────────────────────────────────────────

TEST(GpsValueChanged, IdenticalValuesAreUnchanged)
{
  EXPECT_FALSE(
      fg::GpsValueChanged(53.089172501, 6.169298185333333, 53.089172501, 6.169298185333333));
}

TEST(GpsValueChanged, DifferingLatitudeIsChanged)
{
  EXPECT_TRUE(
      fg::GpsValueChanged(53.089172502, 6.169298185333333, 53.089172501, 6.169298185333333));
}

TEST(GpsValueChanged, DifferingLongitudeIsChanged)
{
  EXPECT_TRUE(
      fg::GpsValueChanged(53.089172501, 6.169298185333334, 53.089172501, 6.169298185333333));
}

TEST(GpsValueChanged, NanPreviousAlwaysCountsAsChanged)
{
  // First real fix after boot / a NO_FIX gap: prev is NaN (never populated).
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(fg::GpsValueChanged(53.0, 6.0, nan, nan));
}

// ── Field-scale regression: a stuck receiver trips within a few metres,
// a healthy one (real per-sample jitter) never does ──────────────────────
//
// These simulations mirror the ACTUAL OnGnss loop's two-different-resets
// shape (fusion_graph_node.hpp / fusion_graph_node_callbacks_a.cpp): the
// distance accumulator resets only when the reported value changes; the
// rotation accumulator resets every sample, matching rtk_wrongfix_gate.hpp's
// own per-fix reset. Getting this wrong in either direction is exactly the
// 2026-09-21 field regression these tests exist to pin.

TEST(GpsStuckGate, StuckReceiverTripsWithinFieldObservedTravel)
{
  const double lat = 53.089172501;
  const double lon = 6.169298185333333;
  double last_lat = lat;
  double last_lon = lon;
  double wheel_dist_since_value_changed_m = 0.0;
  double abs_dtheta_since_last_sample_rad = 0.0;
  const double per_sample_travel_m = 0.05;  // ~0.5 m/s at 10 Hz.
  const double min_wheel_dist_m = 1.0;
  const double max_yaw_rad = 1.047;

  bool tripped = false;
  int trip_at_sample = -1;
  for (int i = 0; i < 200; ++i)
  {
    wheel_dist_since_value_changed_m += per_sample_travel_m;
    // Receiver never actually changes the value (the bug under test).
    if (fg::GpsValueChanged(lat, lon, last_lat, last_lon))
    {
      wheel_dist_since_value_changed_m = 0.0;
    }
    last_lat = lat;
    last_lon = lon;

    const bool stuck = fg::GpsStuckImplausible(wheel_dist_since_value_changed_m,
                                               abs_dtheta_since_last_sample_rad,
                                               min_wheel_dist_m,
                                               max_yaw_rad);
    abs_dtheta_since_last_sample_rad = 0.0;  // every sample, per OnGnss.
    if (stuck)
    {
      tripped = true;
      trip_at_sample = i;
      break;
    }
  }
  EXPECT_TRUE(tripped);
  // 1.0 m / 0.05 m per sample = 20 samples (2 s at 10 Hz) — fast detection.
  EXPECT_EQ(trip_at_sample, 19);
}

TEST(GpsStuckGate, HealthyReceiverWithPerSampleJitterNeverTrips)
{
  // A genuinely live receiver updates the value (even by a tiny amount) on
  // every sample, so the distance accumulator resets before it can build up.
  double last_lat = 53.089172501;
  double last_lon = 6.169298185333333;
  double wheel_dist_since_value_changed_m = 0.0;
  double abs_dtheta_since_last_sample_rad = 0.0;
  const double per_sample_travel_m = 0.05;
  const double min_wheel_dist_m = 1.0;
  const double max_yaw_rad = 1.047;

  for (int i = 0; i < 200; ++i)
  {
    // Every sample nudges lon by a tiny but genuine amount — like real
    // motion + RTK jitter, never bit-identical to the previous sample.
    const double lat = last_lat;
    const double lon = last_lon + 1e-9;

    wheel_dist_since_value_changed_m += per_sample_travel_m;
    if (fg::GpsValueChanged(lat, lon, last_lat, last_lon))
    {
      wheel_dist_since_value_changed_m = 0.0;
    }
    last_lat = lat;
    last_lon = lon;

    const bool stuck = fg::GpsStuckImplausible(wheel_dist_since_value_changed_m,
                                               abs_dtheta_since_last_sample_rad,
                                               min_wheel_dist_m,
                                               max_yaw_rad);
    abs_dtheta_since_last_sample_rad = 0.0;
    EXPECT_FALSE(stuck) << "sample #" << i;
  }
}

// ── 2026-09-21 field regression: cumulative rotation over a stuck period
// must not permanently disable detection ──────────────────────────────────
//
// Field data (session gnss-rc1-test-20260921-0933): /gps/fix froze at t≈1s
// and stayed frozen for the whole ~47 s session; a transit turn starting at
// t≈24s integrated a NET yaw past 170° by the end. abs_dtheta accumulates
// |rate|*dt, so an accumulator that resets only on value-change (the bug)
// sums EVERY inter-message rotation across the ENTIRE stuck period, not just
// the turn itself — even a moderate turn accumulates past a 60° stand-down
// budget within a couple of seconds if nothing ever resets it, and once past
// it, it can only ever grow further while the value stays stuck. The fix
// resets it every GPS message instead (one inter-fix interval, matching
// rtk_wrongfix_gate.hpp), so a REALISTIC single-interval rotation (this
// session's /gps/fix published at ~1 Hz — 46 distinct stamps over 44.95 s;
// even at that comparatively low rate a fast turn is degrees, not tens of
// degrees, per interval) never approaches the stand-down budget on its own,
// and detection keeps firing continuously through the turn instead of going
// silent.
TEST(GpsStuckGate, CumulativeRotationOverStuckPeriodDoesNotDisableDetection)
{
  const double lat = 53.089172501;
  const double lon = 6.169298185333333;
  double last_lat = lat;
  double last_lon = lon;
  double wheel_dist_since_value_changed_m = 0.0;
  double abs_dtheta_since_last_sample_rad = 0.0;
  const double min_wheel_dist_m = 1.0;
  const double max_yaw_rad = 1.047;

  auto tick = [&](double wheel_dist_delta_m, double dtheta_delta_rad) -> bool
  {
    wheel_dist_since_value_changed_m += wheel_dist_delta_m;
    if (fg::GpsValueChanged(lat, lon, last_lat, last_lon))
    {
      wheel_dist_since_value_changed_m = 0.0;
    }
    last_lat = lat;
    last_lon = lon;
    abs_dtheta_since_last_sample_rad += dtheta_delta_rad;

    const bool stuck = fg::GpsStuckImplausible(wheel_dist_since_value_changed_m,
                                               abs_dtheta_since_last_sample_rad,
                                               min_wheel_dist_m,
                                               max_yaw_rad);
    abs_dtheta_since_last_sample_rad = 0.0;
    return stuck;
  };

  // Straight driving crosses the distance threshold first (6 messages @
  // 0.2 m each = 1.2 m, strictly over the 1.0 m min_wheel_dist_m threshold;
  // well under the field's 2.7-4.5 m).
  bool tripped_before_turn = false;
  for (int i = 0; i < 6; ++i)
  {
    if (tick(0.2, 0.0))
    {
      tripped_before_turn = true;
    }
  }
  EXPECT_TRUE(tripped_before_turn);

  // ~171 deg net turn over 20 GPS messages — the field magnitude — but
  // spread per-message, matching how the accumulator is actually fed.
  // 171/20 ≈ 8.6 deg per message, nowhere near the 60 deg stand-down
  // budget for any SINGLE message (even at this session's field-confirmed
  // ~1 Hz publish rate, a 20-message turn spans ~20 s, well within the
  // ~23 s the field turn actually took).
  int stuck_count_during_turn = 0;
  for (int i = 0; i < 20; ++i)
  {
    if (tick(0.0, 0.15))  // ~8.6 deg this message; cumulative total ~171 deg.
    {
      ++stuck_count_during_turn;
    }
  }
  // Every message during the turn should still correctly detect the stuck
  // value — the whole point of the fix. The pre-fix accumulator would have
  // stood down for the entire turn (and everything after it).
  EXPECT_EQ(stuck_count_during_turn, 20);

  // Straight driving resumes; value is STILL stuck. Detection must keep
  // firing — this is what the field regression broke (it never fired again
  // after the turn began).
  EXPECT_TRUE(tick(0.2, 0.0));
}

TEST(GpsStuckGate, ASingleViolentSingleMessageRotationStandsDownThatMessageOnly)
{
  // The stand-down's actual purpose: one message whose OWN inter-fix
  // rotation is implausibly large (a genuine in-place spin between two GPS
  // messages) stands down for that message, but does not linger — the very
  // next message, with normal rotation, evaluates normally again.
  const double lat = 53.089172501;
  const double lon = 6.169298185333333;
  double last_lat = lat;
  double last_lon = lon;
  double wheel_dist_since_value_changed_m = 0.0;
  double abs_dtheta_since_last_sample_rad = 0.0;
  const double min_wheel_dist_m = 1.0;
  const double max_yaw_rad = 1.047;

  auto tick = [&](double wheel_dist_delta_m, double dtheta_delta_rad) -> bool
  {
    wheel_dist_since_value_changed_m += wheel_dist_delta_m;
    if (fg::GpsValueChanged(lat, lon, last_lat, last_lon))
    {
      wheel_dist_since_value_changed_m = 0.0;
    }
    last_lat = lat;
    last_lon = lon;
    abs_dtheta_since_last_sample_rad += dtheta_delta_rad;
    const bool stuck = fg::GpsStuckImplausible(wheel_dist_since_value_changed_m,
                                               abs_dtheta_since_last_sample_rad,
                                               min_wheel_dist_m,
                                               max_yaw_rad);
    abs_dtheta_since_last_sample_rad = 0.0;
    return stuck;
  };

  // Cross the distance threshold first (6 x 0.2 m = 1.2 m, strictly over
  // the 1.0 m min_wheel_dist_m threshold).
  for (int i = 0; i < 6; ++i)
  {
    tick(0.2, 0.0);
  }
  // One message with an implausibly large single-interval rotation (90 deg
  // between two GPS messages) stands down for that message only.
  EXPECT_FALSE(tick(0.0, 1.6));
  // The next message, normal rotation, evaluates normally — still stuck.
  EXPECT_TRUE(tick(0.0, 0.0));
}
