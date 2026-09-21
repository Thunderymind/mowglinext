// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure GPS payload-value staleness gate, factored out of OnGnss so it is
// unit-testable without ROS/GTSAM (see fusion_graph_node_callbacks_a.cpp).
//
// mowglinext#694/#695 field data (2026-09-20, two independent sessions)
// disproved the assumption that reusing LocalizationMonitorNode's
// DEAD_RECKONING verdict (/mowgli/localization/mode_id) would catch a stuck
// receiver: in both sessions /gps/fix stayed bit-for-bit frozen at one
// lat/lon for the entire ~50-70 s session while the robot demonstrably moved
// (2.7-4.5 m of wheel-integrated distance), yet localization_monitor_node
// reported RTK_FLOAT/RTK_FIXED (never DEAD_RECKONING) throughout, and
// gnss_observation_tracker_'s own receipt-stamp dedup also never caught it
// (the receipt stamp keeps genuinely advancing). The likely explanation:
// the receiver's typed position_observation_sequence (which
// NavSatStatusAssociation pairs against) advances on the receiver's own
// internal acceptance cadence, not on the solved position actually changing
// — so every layer that trusts "a new sequence/stamp arrived" as proof of a
// new OBSERVATION is blind to a receiver whose solver is stuck while its
// bookkeeping keeps ticking. Only a direct comparison of the reported
// lat/lon VALUE against physical evidence (wheel motion) can catch this.
//
// CRITICAL DESIGN NOTE — this accumulator's reset condition is the OPPOSITE
// of rtk_wrongfix_gate.hpp's, and must stay that way. The wrongfix gate's
// accumulators reset on EVERY fix (accept or reject) because they bound "how
// far could the chassis have moved since the last fix" — resetting only on
// accept was exactly the reverted GnssMobileGate's bug (CLAUDE.md "What NOT
// to Do": unbounded growth, GPS locked out forever). This gate's
// accumulators track the OPPOSITE quantity — "how far has the chassis moved
// since the reported GPS VALUE last changed" — and must keep growing across
// repeated rejections of the same stuck value; resetting them on every call
// would make a genuinely stuck receiver undetectable (the very case this
// gate exists to catch). They are still bounded: any genuine change in the
// reported value — including the receiver recovering — resets them
// immediately, and a prolonged stuck period is architecturally identical to
// the GNSS-outage handling this node already has (root CLAUDE.md Invariant
// 1's LiDAR-anchor outage warm-up).

#pragma once

#include <cmath>

namespace fusion_graph
{

// True if the reported GPS value has not moved for longer than physically
// plausible: `wheel_dist_since_value_changed_m` of chassis travel accumulated
// since the last time the reported lat/lon actually differed from the
// previous sample, exceeding `min_wheel_dist_m`. Stands down (returns false)
// above `max_yaw_rad` of accumulated rotation since the value last changed —
// mid-turn, net translation is not a reliable "should have moved" signal
// (same reasoning as the dig detector's gyro-gated turn exclusion, root
// CLAUDE.md Invariant 16), and this must stay a translation-only check so it
// does not fire on a robot legitimately pivoting in place.
inline bool GpsStuckImplausible(double wheel_dist_since_value_changed_m,
                                double abs_dtheta_since_value_changed_rad,
                                double min_wheel_dist_m,
                                double max_yaw_rad)
{
  if (abs_dtheta_since_value_changed_rad > max_yaw_rad)
  {
    return false;
  }
  return wheel_dist_since_value_changed_m > min_wheel_dist_m;
}

// True if (lat, lon) is a genuine change from (prev_lat, prev_lon) — bit-exact
// comparison is intentional: the observed failure mode repeats the identical
// floating-point value sample after sample, and any real new fix (even
// millimetre-level RTK jitter) will differ in the low bits. NaN inputs (no
// fix yet) are never equal to themselves or anything else, so the first real
// fix after a NO_FIX gap always counts as a change.
inline bool GpsValueChanged(double lat, double lon, double prev_lat, double prev_lon)
{
  return lat != prev_lat || lon != prev_lon;
}

}  // namespace fusion_graph
