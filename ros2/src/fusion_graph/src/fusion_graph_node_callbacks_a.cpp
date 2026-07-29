// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// FusionGraphNode — input callbacks: wheel/imu/gnss. (The node implementation is split across
// several translation units to keep each file within the project's 600-line budget; all share
// fusion_graph_node.hpp + fusion_graph_node_util.hpp.)

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "fusion_graph/fusion_graph_node.hpp"
#include "fusion_graph/fusion_graph_node_util.hpp"
#include "fusion_graph/rtk_wrongfix_gate.hpp"

namespace fusion_graph
{

void FusionGraphNode::OnWheelOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);
  // Latest forward velocity for the local-frame DR integration in OnImu.
  // twist.linear.y is non-holonomically locked to 0 by hardware_bridge
  // (tight vy covariance) — we mirror that by only integrating vx.
  wheel_vx_ = msg->twist.twist.linear.x;
  wheel_wz_ = msg->twist.twist.angular.z;
  wheel_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
  if (std::abs(wheel_vx_) <= gps_pivot_max_vx_mps_ &&
      std::abs(wheel_wz_) >= gps_pivot_min_wz_rad_per_s_)
  {
    last_pivot_motion_stamp_ = this->now();
  }
  if (last_wheel_stamp_)
  {
    double dt = (stamp - *last_wheel_stamp_).seconds();
    if (dt > 0.0 && dt < 1.0)
    {
      graph_->AddWheelTwist(msg->twist.twist.linear.x,
                            msg->twist.twist.linear.y,
                            msg->twist.twist.angular.z,
                            dt);
      // Track wheel-derived distance since the last GPS fix for the
      // RTK wrong-fix sanity gate in OnGnss. Speed-magnitude × dt is
      // the right scalar — direction doesn't matter for the threshold
      // test, only how far the chassis travelled.
      const double speed = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
      wheel_dist_since_last_gps_m_ += speed * dt;
    }
  }
  last_wheel_stamp_ = stamp;
}

void FusionGraphNode::OnImu(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);
  const double scaled_gz = 1.024 * msg->angular_velocity.z;
  double corrected_gz = scaled_gz;
  if (last_imu_stamp_)
  {
    double dt = (stamp - *last_imu_stamp_).seconds();
    if (dt > 0.0 && dt < 1.0)
    {
      graph_->AddGyroDelta(scaled_gz, dt);
      // Keep local dead reckoning aligned with the graph's wheel-dominant
      // heading model.  The FC IMU bridge can retain a small residual yaw
      // bias, which otherwise slowly turns the local odom/TF frame even on a
      // straight drive.  A stale wheel sample, or a detected wheel slip,
      // falls back to the bias-corrected gyro instead.
      // position uses the latest wheel vx with the just-updated yaw.
      // Sub-cm/sub-° accuracy per IMU sample at typical 91 Hz / 0.5 m/s.
      const double gz = graph_->CorrectedGyroZ(scaled_gz);
      corrected_gz = gz;
      // Slip veto (see header): if the wheels claim a yaw rate the
      // gyro doesn't see, the chassis is being skated, not driven —
      // its forward velocity is phantom. Drop the translation for
      // this sample; yaw still integrates from the gyro, which is the
      // honest source during a slipping pivot. Without this the odom
      // frame accumulates the fictitious forward motion unbounded.
      const bool dr_slip = std::abs(wheel_vx_) < dr_slip_max_vx_mps_ &&
                           std::abs(wheel_wz_ - gz) > dr_slip_wheel_min_rad_per_s_ &&
                           std::abs(gz) < dr_slip_gyro_max_rad_per_s_ &&
                           std::abs(wheel_wz_) > dr_slip_wheel_min_rad_per_s_;
      const bool wheel_yaw_fresh = last_wheel_stamp_.has_value() && wheel_yaw_.has_value() &&
                                   std::abs((stamp - *last_wheel_stamp_).seconds()) < 0.20;
      const bool use_wheel_yaw = false;
      const double dr_wz = gz;
      const double vx_eff = dr_slip ? 0.0 : wheel_vx_;
      double dr_dyaw = gz * dt;
      if (use_wheel_yaw)
      {
        // Use the encoder pose's actual heading change. This preserves any
        // local graph-to-DR rebase already applied to dr_yaw_ while avoiding
        // the wheel twist-rate integration error.
        if (consumed_wheel_yaw_)
        {
          const double raw_delta = *wheel_yaw_ - *consumed_wheel_yaw_;
          dr_dyaw = std::atan2(std::sin(raw_delta), std::cos(raw_delta));
        }
        else
        {
          dr_dyaw = 0.0;
        }
        consumed_wheel_yaw_ = wheel_yaw_;
      }
      else if (wheel_yaw_fresh)
      {
        // Do not apply a rejected slip interval retroactively when the
        // wheel and gyro agree again on a later sample.
        consumed_wheel_yaw_ = wheel_yaw_;
      }
      {
        // tf_state_mu_: dr_* is read concurrently by TfBroadcastLoop.
        std::lock_guard<std::mutex> lock(tf_state_mu_);
        dr_yaw_ = std::atan2(std::sin(dr_yaw_ + dr_dyaw), std::cos(dr_yaw_ + dr_dyaw));
        dr_x_ += vx_eff * std::cos(dr_yaw_) * dt;
        dr_y_ += vx_eff * std::sin(dr_yaw_) * dt;
        // Cache the velocities that produced this step so the TF broadcast can
        // honestly forward-propagate the pose by tf_publish_lead_s_.
        dr_last_gz_ = dr_wz;
        dr_last_vx_eff_ = vx_eff;
      }
      // Accumulate |Δθ| since the last accepted GPS for the wrong-fix
      // gate. A stationary pivot sweeps the GPS antenna by lever_arm
      // × Δθ in the map frame; without this term the gate sees a
      // pure-sweep jump as if it were a phantom translation and
      // rejects every legitimate fix.
      abs_dtheta_since_last_gps_rad_ += std::abs(dr_dyaw);
    }
  }
  last_imu_stamp_ = stamp;
  // Feed the high-rate extrapolator (item #15) too. Safe even when
  // fast_pose_timer_ is null — the extrapolator is just a value
  // cache.
  pose_extrap_.OnImuGyro(stamp.seconds(), corrected_gz);
}


void FusionGraphNode::OnGnssStatus(mowgli_interfaces::msg::GnssStatus::ConstSharedPtr msg)
{
  gnss_rtk_fixed_ =
      msg->fix_type == mowgli_interfaces::msg::GnssStatus::FIX_TYPE_RTK_FIXED &&
      msg->differential_corrections && msg->corrections_active;
  last_gnss_status_stamp_ = this->now();
}

void FusionGraphNode::OnGnss(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
    return;
  // First valid fix gates the dock-arrival pose seed below. Without
  // this, a robot that boots already docked could anchor on the dock
  // before GPS is ready and walk the graph over once GPS arrives.
  gps_seen_once_ = true;
  // hardware_bridge may publish its one-shot "already charging" status before
  // the first GPS fix arrives.  If it does not publish another status sample,
  // OnHardwareStatus never gets a chance to satisfy its dock+GPS seed gate and
  // the autoloaded graph heading survives on the dock.  Complete that pending
  // dock seed from the first valid GPS callback instead.
  if (last_is_charging_valid_ && last_is_charging_ && !dock_seeded_this_session_)
  {
    SeedFromDockPose();
    dock_seeded_this_session_ = true;
  }
  if (datum_lat_ == 0.0 && datum_lon_ == 0.0)
  {
    // Self-seed datum from first valid fix. Not ideal — operator should
    // set datum in mowgli_robot.yaml — but keeps the node from refusing
    // to start during sim/dev.
    datum_lat_ = msg->latitude;
    datum_lon_ = msg->longitude;
    datum_cos_lat_ = std::cos(datum_lat_ * M_PI / 180.0);
    RCLCPP_WARN(get_logger(),
                "fusion_graph: datum self-seeded from first fix "
                "(%.9f, %.9f) — set datum_lat/lon explicitly",
                datum_lat_,
                datum_lon_);
  }

  double mx, my;
  LatLonToMap(msg->latitude, msg->longitude, mx, my);

  const rclcpp::Time meas_stamp(msg->header.stamp);
  // The bridge preserves the receiver epoch in header.stamp. Bind this raw
  // antenna position to the graph state from that epoch; do not project it
  // forward, because a projection depends on the turn radius and can invent
  // metre-scale position circles during tight steering.
  const auto historical_node = meas_stamp.nanoseconds() != 0
                                   ? graph_->FindNodeAtOrBefore(meas_stamp.seconds())
                                   : std::nullopt;

  // RTK factors are Huber-robust downstream. Do not pre-filter a receiver
  // epoch by comparing it with wheel distance: at 1 Hz a real curved path
  // (especially at a left/right transition) can exceed that chord heuristic
  // even while the LC29H is correctly RTK-Fixed. Dropping those samples leaves
  // the graph wheel/IMU-only at exactly the point where it needs RTK most.

  // covariance[0] is variance of east; take sqrt for sigma. Use the
  // diagonal mean for a single sigma_xy (factor model is isotropic).
  const double var_x = msg->position_covariance[0];
  const double var_y = msg->position_covariance[4];
  double sigma = std::sqrt(0.5 * (var_x + var_y));
  // SAFETY: a fix with UNKNOWN covariance, or a zero/non-finite reported σ, has
  // NO trustworthy accuracy. The old code set σ=-1.0 here as a "floor" sentinel,
  // but graph_manager then clamps σ UP to gps_sigma_floor (3 mm) — so a fix with
  // no known accuracy (e.g. a standalone/SBAS position when the driver leaves
  // covariance UNKNOWN) was fused as a GnssLeverArmFactor at RTK-Fixed precision,
  // yanking map→odom by metres toward a garbage position (the next good fix then
  // looks like a jump and the wrong-fix gate drops it, locking GPS out). Reject
  // such fixes outright — wheel/gyro/COG keep localising — instead of trusting
  // them. (navsat_to_absolute_pose_node guards covariance_type the same way, but
  // it no longer feeds the localizer.)
  if (msg->position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      if (msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
          sigma = 0.02; // RTK Fixed
      } else if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX) {
          sigma = 0.1; // RTK Float
      } else if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
          sigma = 1.0;
      }
  }

  if (!std::isfinite(sigma) || sigma <= 0.0)
  {
    graph_->RecordGpsRejectWrongFix();
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         5000,
                         "fusion_graph: GPS fix with unknown/zero covariance "
                         "(cov_type=%u, var_x=%.4g, var_y=%.4g) — rejected (would "
                         "otherwise be trusted as 3 mm RTK precision)",
                         msg->position_covariance_type,
                         var_x,
                         var_y);
    last_gps_sigma_ = -1.0;  // no usable σ this epoch (keyframe gate stays closed)
    return;
  }
  if (gps_sigma_speed_coeff_ > 0.0 || gps_sigma_omega_coeff_ > 0.0)
  {
    // Inflate the GPS σ with chassis speed and turning rate. The receiver covariance
    // reports instantaneous fix precision but ignores motion-induced position error:
    // 1) Translation displacement: GPS latency × linear velocity v
    // 2) Rotational sweep displacement: GPS latency × turning rate |w| × r_sweep
    const double v = std::abs(wheel_vx_);
    const double w = std::abs(wheel_wz_);
    const double r_sweep = std::max(lever_arm_radius_m_, 0.5);
    const double speed_term = gps_sigma_speed_coeff_ * v;
    const double turn_term = gps_sigma_omega_coeff_ * w * r_sweep;
    const double motion_term = speed_term + turn_term;
    sigma = std::sqrt(sigma * sigma + motion_term * motion_term);
  }

  // SAFETY: max-σ reject. A fix this imprecise is a garbage / standalone
  // position; fusing it (even at its honest large σ) is not worth the risk of a
  // wrong-fix step. Disabled at 0 (default) so genuine RTK-Float — which the
  // multi-minute ride-through depends on — is never rejected; operators size it
  // generously above their worst Float σ when they want the extra guard.
  if (gps_max_sigma_reject_m_ > 0.0 && sigma > gps_max_sigma_reject_m_)
  {
    graph_->RecordGpsRejectWrongFix();
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         5000,
                         "fusion_graph: GPS σ=%.3f m > reject threshold %.3f m — "
                         "sample dropped",
                         sigma,
                         gps_max_sigma_reject_m_);
    last_gps_sigma_ = -1.0;
    return;
  }
  // Latch the most-recent valid GPS σ for the keyframe-capture quality gate
  // (only capture when the fix is mm-accurate). <0 means no usable σ.
  last_gps_sigma_ = sigma;

  // Robust noise model on GPS — applied unconditionally now (was
  // RTK-Float only). Field measurement 2026-05-17 (gps_stability.py,
  // 10 min stationary on RTK-Fixed 100 %) showed even Fixed solutions
  // carry σ ≈ 8-12 mm of multipath/constellation noise and the
  // occasional ~3 cm wrong-fix outlier — well above the 3 mm σ_floor.
  // Huber at k=1.345 σ keeps Gaussian inliers fully efficient and
  // smoothly downweights the rare wrong-fix outlier even if the
  // pre-graph gate above doesn't catch it (e.g. first sample of a
  // session, or a slow drift that builds up to >5 cm without a
  // detectable wheel discrepancy).
  const bool status_fresh = last_gnss_status_stamp_ &&
      (this->now() - *last_gnss_status_stamp_).seconds() < cog_rtk_max_age_s_;
  const bool rtk_fixed = status_fresh ? gnss_rtk_fixed_ :
      msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  if (!rtk_fixed)
  {
    // RTK Float is a degraded-navigation status only. It must never change
    // absolute position: its correlated ambiguity/multipath error caused the
    // one-Hz lateral drift. Wheel odometry and gyro carry the pose until a
    // verified RTK-Fixed epoch returns.
    last_gps_sigma_ = -1.0;
    rtk_fixed_streak_ = 0;
    return;
  }
  // Track the freshness of RTK-Fixed for the scan-match yield gate. Updated
  // even while docked (GPS factors are suppressed below, but the freshness is
  // still the honest signal of whether absolute position is available).
  if (rtk_fixed)
  {
    last_rtk_fixed_stamp_ = this->now();
  }
  // Debounce counter for keyframe capture: only capture after RTK-Fixed has
  // held for several consecutive epochs (a single carrSoln Fixed flicker can
  // otherwise freeze a slightly-off anchor that poisons every later match).
  rtk_fixed_streak_ = rtk_fixed ? (rtk_fixed_streak_ + 1) : 0;
  // During the dock approach, hold position through the RTK fixed↔float
  // per-epoch flicker: drop non-Fixed epochs entirely so the dock controller's
  // target doesn't jump between cm-accurate Fixed and dm-noisy Float (the
  // flicker, not the controller, was the 2026-06-10 divergence trigger). Fixed
  // epochs update the graph normally below; the is_charging suppression takes
  // over once docked.
  // ...but never starve a yet-uninitialized graph of its bootstrap GPS seed.
  if (gate_float_gps_during_docking_ && !rtk_fixed && DockingApproachActive() &&
      graph_->IsInitialized())
  {
    return;
  }
  // Suppress GPS factors while the robot is on the dock.
  //
  // When `is_charging=true`, the operator-calibrated dock_pose (anchored
  // by SeedFromDockPose with σ≈10 cm) is the authoritative ground truth
  // on the robot's position. Even RTK-Fixed GPS only matches the dock
  // pose to 1-3 cm at best — and routinely shifts 5-30 cm across F9P
  // re-acquisitions on different ambiguity sets between sessions. Every
  // GnssLeverArmFactor (σ≈5 mm, ~7 Hz) accumulated while docked pulls
  // the trajectory toward the live GPS measurement and away from
  // dock_pose, so after a minute or two the EKF has walked off the
  // anchor by 10-30 cm.
  //
  // Robot on dock = stationary chassis with known position; we don't
  // need GPS to know where it is. Skipping QueueGnss preserves the
  // dock_pose anchor exactly. When the robot undocks, the next OnGnss
  // (now with is_charging=false) resumes injecting GPS factors and the
  // trajectory transitions back to GPS-tracking. seed_xy_ is also
  // skipped because TrySeedInitialPose should use dock_pose, not GPS,
  // to bootstrap if the graph somehow becomes uninitialised.
  if (last_is_charging_valid_ && last_is_charging_)
  {
    // On the dock the dock_pose is authoritative ground truth — the dock does
    // not move. The previous approach injected a WEAK live-GPS factor here for
    // well-posedness, but a ~7 Hz stream of σ≈0.5 m factors at the live RTK
    // position (5-30 cm off dock_pose, and drifting yaw with nothing holding it)
    // out-votes the single one-shot dock prior and WALKS the docked pose off the
    // anchor over the charge dwell (field 2026-06-10: 11.5 cm + 53° walk, which
    // then made re-docking aim at the wrong target — the "never dock twice"
    // symptom). Instead, periodically re-assert a firm absolute anchor at the
    // FULL dock_pose (x, y, AND yaw). With no live GPS to drag it, the
    // accumulated dock priors keep iSAM2 well-posed AND pin the docked pose
    // exactly where the operator calibrated it, holding both position and yaw.
    // seed_xy_ is still NOT updated (TrySeedInitialPose bootstraps from dock_pose).
    if (graph_->IsInitialized())
    {
      if (auto snap = graph_->LatestSnapshot())
      {
        // Re-anchor each NEW node exactly once (nodes appear ~1/5 s while
        // stationary), so the prior count tracks the node count and stays
        // bounded by the sliding window rather than piling several priors onto
        // the same stationary node.
        if (snap->node_index != last_dock_reanchor_node_)
        {
          const gtsam::Pose2 dock(dock_pose_x_, dock_pose_y_, dock_pose_yaw_);
          graph_->ForceAnchor(snap->node_index,
                              dock,
                              dock_reanchor_sigma_xy_m_,
                              std::max(dock_pose_yaw_sigma_rad_, 0.035));
          last_dock_reanchor_node_ = snap->node_index;
        }
      }
    }
    TrySeedInitialPose();
    return;
  }

  // With the factor attached to the receiver-epoch graph node, RTK position
  // remains valid at every turn radius (and at rest). Do not discard a Fixed
  // observation merely because the chassis is turning: that leaves a long
  // wheel/IMU-only arc precisely where heading changes sign.
  // A verified RTK-Fixed epoch is the absolute reference. Applying the
  // centimetre-scale Huber kernel to it causes a perfectly real correction
  // after a curved segment to be treated as an outlier, leaving the current
  // trajectory unconstrained and its covariance to grow. Keep robustness for
  // lower-quality fixes only.
  if (graph_->IsInitialized() && meas_stamp.nanoseconds() != 0 && !historical_node)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "fusion_graph: dropping RTK epoch with no historical graph node");
    return;
  }
  graph_->QueueGnss(mx, my, sigma, /*robust=*/false, historical_node);
  // Only a factor that passed every quality/docking gate above is allowed to
  // move the map frame.  Raw GNSS reception is not sufficient: rejected or
  // missing GPS must leave map→odom rigid and let wheel/IMU odometry carry the
  // map continuously.
  last_gnss_factor_stamp_ = this->now();
  if (!rtk_fixed)
  {
    last_float_gnss_factor_stamp_ = last_gnss_factor_stamp_;
  }
  ++gnss_factor_sequence_;
  if (historical_node && meas_stamp.nanoseconds() != 0)
  {
    pending_gnss_anchor_ = PendingGnssAnchor{
        gnss_factor_sequence_, *historical_node, meas_stamp.nanoseconds()};
  }
  seed_xy_ = gtsam::Vector2(mx, my);
  // Latch whether the most recent seed came from RTK-Fixed so the next
  // graph initialization can use a tight prior matching that quality.
  // Stale once seeded but TrySeedInitialPose only fires once per
  // (re)initialization, so the freshness window is the same as the
  // seed itself.
  seed_xy_rtk_fixed_ = rtk_fixed;

  // RTK-Fixed override of an autoloaded init: the persisted graph's last
  // node is almost always the dock (auto-save fires on dock arrival), so
  // booting away from the dock leaves IsInitialized()=true at the wrong
  // pose and TrySeedInitialPose() short-circuits — GPS observations then
  // fight the dock prior for many seconds before the trajectory walks
  // over. RTK-Fixed is sub-cm and trustworthy: re-anchor the latest
  // loaded node at the GPS pose with a tight prior. One-shot per boot.
  //
  // BUT — if the robot is physically on the dock at boot (is_charging),
  // SeedFromDockPose owns the anchor — it's the operator-calibrated
  // ground truth on the robot's position, independent of how the F9P's
  // RTK integer ambiguities happened to land this session. The tight
  // RTK override (σ=5mm) would dominate the looser dock seed
  // (σ=10cm) and pull the trajectory to the live GPS, defeating the
  // whole point of having a persisted dock_pose. So:
  //   * If /hardware_bridge/status hasn't been seen yet
  //     (!last_is_charging_valid_) — defer; the next /gps/fix tick
  //     will re-check once we know whether we're docked.
  //   * If docked, suppress this override entirely (latch one-shot
  //     done) and let SeedFromDockPose anchor the graph.
  //   * Otherwise (off-dock, status valid) proceed as before.
  if (false && rtk_fixed && autoload_succeeded_ && !rtk_autoload_override_done_ && graph_->IsInitialized() &&
      last_is_charging_valid_ && !last_is_charging_)
  {
    auto snap = graph_->LatestSnapshot();
    if (snap)
    {
      const double dx = mx - snap->pose.x();
      const double dy = my - snap->pose.y();
      const double dist = std::hypot(dx, dy);
      if (dist > rtk_autoload_override_threshold_m_)
      {
        // Use the freshest yaw seed if we have one (COG/mag have already
        // populated seed_yaw_ if they're alive); otherwise keep the
        // autoloaded yaw — it's better than nothing and the next COG
        // sample will pull it.
        const double yaw = seed_yaw_.value_or(snap->pose.theta());
        const gtsam::Pose2 anchor(mx, my, yaw);
        // σ=5mm matches RTK-Fixed reported precision; σ_yaw 5° is loose
        // enough to let COG correct it without fighting if the
        // autoloaded yaw is wrong.
        graph_->ForceAnchor(snap->node_index, anchor, 0.005, 5.0 * M_PI / 180.0);
        // ForceAnchor shifts latest_.pose without bumping node_index;
        // OnTimer's "node_index changed" check would miss it, leaving
        // map→odom anchored at the pre-override correction. Force a
        // refresh so the next OnTimer recomputes against fresh dr_*.
        t_map_odom_anchor_valid_ = false;
        rtk_autoload_override_done_ = true;
        RCLCPP_WARN(get_logger(),
                    "fusion_graph: RTK-Fixed override of autoloaded pose — "
                    "re-anchored node %lu (%.2f, %.2f) → (%.2f, %.2f), Δ=%.2f m",
                    static_cast<unsigned long>(snap->node_index),
                    snap->pose.x(),
                    snap->pose.y(),
                    mx,
                    my,
                    dist);
      }
      else
      {
        // Within threshold — autoload is consistent with RTK, no
        // override needed. Latch so we don't keep checking.
        rtk_autoload_override_done_ = true;
      }
    }
  }

  TrySeedInitialPose();
}

}  // namespace fusion_graph
