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

TEST(GpsStuckGate, StuckReceiverTripsWithinFieldObservedTravel)
{
  // Mirrors the actual OnGnss loop shape: accumulate wheel distance every
  // sample, reset only when the reported value changes.
  const double lat = 53.089172501;
  const double lon = 6.169298185333333;
  double last_lat = lat;
  double last_lon = lon;
  double wheel_dist_since_value_changed_m = 0.0;
  double abs_dtheta_since_value_changed_rad = 0.0;
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
      abs_dtheta_since_value_changed_rad = 0.0;
    }
    last_lat = lat;
    last_lon = lon;

    if (fg::GpsStuckImplausible(wheel_dist_since_value_changed_m,
                                abs_dtheta_since_value_changed_rad,
                                min_wheel_dist_m,
                                max_yaw_rad))
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
  // every sample, so the accumulator resets before it can build up.
  double last_lat = 53.089172501;
  double last_lon = 6.169298185333333;
  double wheel_dist_since_value_changed_m = 0.0;
  double abs_dtheta_since_value_changed_rad = 0.0;
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
      abs_dtheta_since_value_changed_rad = 0.0;
    }
    last_lat = lat;
    last_lon = lon;

    EXPECT_FALSE(fg::GpsStuckImplausible(wheel_dist_since_value_changed_m,
                                         abs_dtheta_since_value_changed_rad,
                                         min_wheel_dist_m,
                                         max_yaw_rad))
        << "sample #" << i;
  }
}
