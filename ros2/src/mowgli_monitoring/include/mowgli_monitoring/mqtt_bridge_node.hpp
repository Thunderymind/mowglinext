// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// SPDX-License-Identifier: GPL-3.0
/**
 * @file mqtt_bridge_node.hpp
 * @brief MqttBridgeNode: bridges key ROS2 topics to MQTT for external
 *        monitoring (mobile app, Home Assistant, etc.).
 *
 * Architecture
 * ------------
 * The node owns an `IMqttClient` interface.  At startup it tries to construct
 * a `MosquittoMqttClient` (requires libmosquitto at link time).  If the build
 * was performed without mosquitto the `StubMqttClient` is used instead, which
 * simply logs every publish/subscribe call at DEBUG level.
 *
 * ROS2 → MQTT
 *   /hardware_bridge/status              → <prefix>/status             (JSON)
 *   /hardware_bridge/power               → <prefix>/power              (JSON)
 *   /hardware_bridge/emergency           → <prefix>/emergency          (JSON)
 *   /wheel_odom                          → <prefix>/position   (JSON: x, y, theta, odom frame) —
 * rate-limited /diagnostics                         → <prefix>/diagnostics        (JSON summary)
 *   /behavior_tree_node/high_level_status → <prefix>/high_level_status (JSON) — retained
 *   /gps/fix                             → <prefix>/gps        (JSON: lat/lon/alt) — rate-limited
 *   /gps/status                          → <prefix>/rtk_status (JSON) — retained; the SAME
 *                                           mowgli_interfaces/msg/GnssStatus + gnss_status_utils
 *                                           helpers the LED ring and behavior tree use, so this
 *                                           can never disagree with what the robot's own ring or
 *                                           GUI "GPS %" badge shows.
 *   /map_server_node/get_mowing_area     → <prefix>/area_boundary (JSON) — retained; polled every
 *                                           ~10s (piggybacks on the existing on_timer() tick, no
 *                                           separate timer), republished only when it actually
 *                                           changed. Datum + map-frame (metres, X=east/Y=north)
 *                                           polygon geometry for every mowing area, so an external
 *                                           consumer can render a boundary/obstacle overview
 *                                           without needing the GUI's own map stack. Independent
 *                                           of <prefix>/areas' own index/name polling (mowglinext
 *                                           PR #638) — the two currently poll the same service
 *                                           separately; consolidating them is a natural follow-up
 *                                           once #638 lands, not done here to avoid touching that
 *                                           in-flight PR's branch.
 *   (connection state)                   → <prefix>/available  ("online"/"offline", retained, LWT)
 *
 * MQTT → ROS2
 *   <prefix>/command → /behavior_tree_node/high_level_control service call
 *                       (payload: ASCII decimal uint8, e.g. "1" — not a raw byte)
 *
 * See docs/MQTT_CONTROL.md for the full JSON schema of every topic above.
 *
 * Parameters
 * ----------
 * mqtt_host          string  "localhost"  — overridden from mowgli_robot.yaml (Invariant 15 /
 * mqtt_port          int     1883            GUI Settings → MQTT) by full_system.launch.py;
 * mqtt_username      string  ""              these package-share defaults only apply when the
 * mqtt_password      string  ""              node is run standalone (e.g. in tests).
 * mqtt_client_id     string  "mowgli_ros2"
 * mqtt_topic_prefix  string  "mowgli"
 * publish_rate       double  1.0   Hz — position/gps update rate limit
 * use_ssl            bool    false
 * datum_lat          double  0.0   — injected from mowgli_robot.yaml by full_system.launch.py,
 * datum_lon          double  0.0     same as map_server_node/navsat_to_absolute_pose_node; used
 *                                     only to label <prefix>/area_boundary's map-frame geometry
 *                                     with the WGS84 origin it's relative to.
 */

#ifndef MOWGLI_MONITORING__MQTT_BRIDGE_NODE_HPP_
#define MOWGLI_MONITORING__MQTT_BRIDGE_NODE_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "mowgli_interfaces/gnss_status_utils.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/gnss_status.hpp"
#include "mowgli_interfaces/msg/high_level_status.hpp"
#include "mowgli_interfaces/msg/map_area.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_interfaces/srv/get_mowing_area.hpp"
#include "mowgli_interfaces/srv/high_level_control.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace mowgli_monitoring
{

// ---------------------------------------------------------------------------
// IMqttClient — pure interface
// ---------------------------------------------------------------------------

/**
 * @brief Minimal MQTT client interface.
 *
 * Implementations provide publish/subscribe/connect/disconnect operations.
 * All methods are noexcept to keep the bridge node simple; implementations
 * must handle failures internally (e.g. log and return false).
 */
class IMqttClient
{
public:
  using MessageCallback = std::function<void(const std::string& topic, const std::string& payload)>;

  virtual ~IMqttClient() = default;

  /**
   * @brief Connect to the broker.
   * @return true on success.
   */
  virtual bool connect() noexcept = 0;

  /**
   * @brief Disconnect from the broker gracefully.
   */
  virtual void disconnect() noexcept = 0;

  /**
   * @brief Publish a message.
   * @param topic   Full MQTT topic string.
   * @param payload UTF-8 payload (typically JSON).
   * @param retain  Whether the broker should retain the last message.
   * @return true if the message was accepted for delivery.
   */
  virtual bool publish(const std::string& topic,
                       const std::string& payload,
                       bool retain = false) noexcept = 0;

  /**
   * @brief Subscribe to a topic pattern.
   * @param topic    MQTT topic filter (may include wildcards + and #).
   * @param callback Invoked on each received message.
   * @return true if the subscription was accepted.
   */
  virtual bool subscribe(const std::string& topic, MessageCallback callback) noexcept = 0;

  /**
   * @brief Drive the client network loop (call regularly).
   *
   * Implementations that maintain an internal event loop (e.g. libmosquitto
   * in synchronous mode) should call their loop function here.
   */
  virtual void spin_once() noexcept = 0;

  /// @return true when currently connected to the broker.
  virtual bool is_connected() const noexcept = 0;
};

// ---------------------------------------------------------------------------
// StubMqttClient — no-op / logging implementation
// ---------------------------------------------------------------------------

/**
 * @brief Stub MQTT client that logs all operations instead of performing them.
 *
 * Used when libmosquitto is not available at build time, or in unit tests.
 */
class StubMqttClient : public IMqttClient
{
public:
  explicit StubMqttClient(rclcpp::Logger logger);

  bool connect() noexcept override;
  void disconnect() noexcept override;
  bool publish(const std::string& topic,
               const std::string& payload,
               bool retain = false) noexcept override;
  bool subscribe(const std::string& topic, MessageCallback callback) noexcept override;
  void spin_once() noexcept override;
  bool is_connected() const noexcept override;

private:
  rclcpp::Logger logger_;
  bool connected_{false};
};

// ---------------------------------------------------------------------------
// MosquittoMqttClient — libmosquitto implementation
// ---------------------------------------------------------------------------

#ifdef MOWGLI_HAS_MOSQUITTO

/**
 * @brief libmosquitto-backed MQTT client.
 *
 * Only compiled when the build system detects libmosquitto
 * (MOWGLI_HAS_MOSQUITTO is set by CMakeLists.txt via find_library).
 */
class MosquittoMqttClient : public IMqttClient
{
public:
  struct Config
  {
    std::string host{"localhost"};
    int port{1883};
    std::string username{};
    std::string password{};
    std::string client_id{"mowgli_ros2"};
    bool use_ssl{false};
    /// Full topic (e.g. "mowgli/available") for the LWT + explicit online/
    /// offline publishes. Empty disables the availability feature entirely.
    std::string availability_topic{};
  };

  explicit MosquittoMqttClient(Config config, rclcpp::Logger logger);
  ~MosquittoMqttClient() override;

  bool connect() noexcept override;
  void disconnect() noexcept override;
  bool publish(const std::string& topic,
               const std::string& payload,
               bool retain = false) noexcept override;
  bool subscribe(const std::string& topic, MessageCallback callback) noexcept override;
  void spin_once() noexcept override;
  bool is_connected() const noexcept override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

#endif  // MOWGLI_HAS_MOSQUITTO

// ---------------------------------------------------------------------------
// MqttBridgeNode
// ---------------------------------------------------------------------------

/**
 * @brief Bridges selected ROS2 topics to/from an MQTT broker.
 *
 * The node accepts an externally created IMqttClient for testability.
 * When constructed without one the factory function `make_default_client()`
 * selects MosquittoMqttClient or StubMqttClient based on build configuration.
 */
class MqttBridgeNode : public rclcpp::Node
{
public:
  /**
   * @brief Primary constructor — creates the MQTT client internally.
   */
  explicit MqttBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Constructor for tests — accepts an externally provided client.
   */
  MqttBridgeNode(std::unique_ptr<IMqttClient> client,
                 const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // Default destructor is sufficient: destroying mqtt_client_ (a
  // MosquittoMqttClient) runs its own destructor, which calls disconnect()
  // — that already publishes the retained "offline" availability payload
  // before actually dropping the connection (see mqtt_bridge_node.cpp).
  ~MqttBridgeNode() override = default;

  // Exposed for testing.
  const std::string& topic_prefix() const
  {
    return topic_prefix_;
  }

  // ---- JSON serialisers ------------------------------------------------
  // Public (pure, static) so gtest can exercise the exact JSON shape without
  // friend-class machinery — same "exposed for testing" convention as
  // DiagnosticsNode's check_*() methods (diagnostics_node.hpp).

  static std::string serialise_status(const mowgli_interfaces::msg::Status& msg);
  static std::string serialise_power(const mowgli_interfaces::msg::Power& msg);
  static std::string serialise_emergency(const mowgli_interfaces::msg::Emergency& msg);
  static std::string serialise_position(const nav_msgs::msg::Odometry& msg);
  static std::string serialise_diagnostics(const diagnostic_msgs::msg::DiagnosticArray& msg);
  static std::string serialise_high_level_status(
      const mowgli_interfaces::msg::HighLevelStatus& msg);
  static std::string serialise_gps(const sensor_msgs::msg::NavSatFix& msg);
  static std::string serialise_rtk_status(const mowgli_interfaces::msg::GnssStatus& msg);

  /**
   * @brief Build the <prefix>/area_boundary payload from a polled area list.
   * @param areas (index, MapArea) pairs, in whatever order they were polled —
   *        NOT necessarily sorted or contiguous (an area can be deleted,
   *        leaving gaps; see docs/MQTT_CONTROL.md's index-staleness caveat).
   *        Navigation-only areas (MapArea::is_navigation_area) must already
   *        be filtered out by the caller, matching <prefix>/areas' own
   *        exclusion (mowglinext PR #638).
   * @param datum_lat / datum_lon WGS84 origin the polygon points (map-frame
   *        metres, X=east/Y=north) are relative to.
   */
  static std::string serialise_area_boundaries(
      const std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>>& areas,
      double datum_lat,
      double datum_lon);

  /// Escape a raw string so it is safe inside a JSON string literal.
  static std::string json_escape(const std::string& raw);

  /**
   * @brief Parse an MQTT command payload into a HighLevelControl command code.
   *
   * Expected payload: a single ASCII decimal integer, e.g. "1" for
   * COMMAND_START — NOT a raw byte. Whitespace/garbage after the number
   * (matched by sscanf's "%d") is tolerated, matching the pre-refactor
   * behaviour; a fully non-numeric payload, one outside [0, 255], or an
   * empty string is rejected.
   * @return true and sets out_command on a valid uint8 payload; false
   *         (out_command left unchanged) otherwise.
   */
  static bool parse_command_payload(const std::string& payload, uint8_t& out_command);

private:
  // ---- Initialisation -------------------------------------------------------

  void declare_parameters();
  void create_mqtt_client();
  void create_subscriptions();
  void create_service_client();
  void create_timer();

  // ---- Area boundary polling (piggybacks on on_timer(), ~every 10s) --------

  /// Kick off a fresh index-0..N poll chain, if one isn't already running.
  void maybe_poll_area_boundaries();
  /// Request GetMowingArea for `index`, then chain to `index + 1` on success.
  void poll_area_boundary_step(
      uint32_t index,
      std::shared_ptr<std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>>>
          accumulated);
  /// Serialise + publish (retained) `accumulated`, but only if it differs
  /// from the last payload actually sent — <prefix>/area_boundary is meant
  /// to be a quiet, retained topic, not a ~10s heartbeat.
  void finish_area_boundary_poll(
      std::shared_ptr<std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>>>
          accumulated);

  // ---- ROS2 subscription callbacks -----------------------------------------

  void on_status(mowgli_interfaces::msg::Status::ConstSharedPtr msg);
  void on_power(mowgli_interfaces::msg::Power::ConstSharedPtr msg);
  void on_emergency(mowgli_interfaces::msg::Emergency::ConstSharedPtr msg);
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void on_diagnostics(diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg);
  void on_high_level_status(mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg);
  void on_gps_fix(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void on_gnss_status(mowgli_interfaces::msg::GnssStatus::ConstSharedPtr msg);

  // ---- MQTT command callback ------------------------------------------------

  void on_mqtt_command(const std::string& topic, const std::string& payload);

  // ---- Timer: network loop + rate-limited position/gps -----------------------

  void on_timer();

  // ---- Helpers --------------------------------------------------------------

  /// Construct the full MQTT topic: "<prefix>/<suffix>".
  std::string full_topic(const std::string& suffix) const;

  // ---- MQTT client ----------------------------------------------------------

  std::unique_ptr<IMqttClient> mqtt_client_;

  // ---- ROS2 interfaces ------------------------------------------------------

  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr sub_status_;
  rclcpp::Subscription<mowgli_interfaces::msg::Power>::SharedPtr sub_power_;
  rclcpp::Subscription<mowgli_interfaces::msg::Emergency>::SharedPtr sub_emergency_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_diagnostics_;
  rclcpp::Subscription<mowgli_interfaces::msg::HighLevelStatus>::SharedPtr sub_high_level_status_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_fix_;
  rclcpp::Subscription<mowgli_interfaces::msg::GnssStatus>::SharedPtr sub_gnss_status_;

  rclcpp::Client<mowgli_interfaces::srv::HighLevelControl>::SharedPtr srv_high_level_;
  rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::SharedPtr srv_get_mowing_area_;

  rclcpp::TimerBase::SharedPtr timer_;

  // ---- Parameters -----------------------------------------------------------

  std::string mqtt_host_{"localhost"};
  int mqtt_port_{1883};
  std::string mqtt_username_{};
  std::string mqtt_password_{};
  std::string mqtt_client_id_{"mowgli_ros2"};
  std::string topic_prefix_{"mowgli"};
  double publish_rate_{1.0};
  bool use_ssl_{false};
  double datum_lat_{0.0};
  double datum_lon_{0.0};

  // ---- Rate-limiting state --------------------------------------------------

  std::optional<nav_msgs::msg::Odometry> pending_odom_{};
  rclcpp::Time last_odom_publish_{0, 0, RCL_ROS_TIME};
  std::optional<sensor_msgs::msg::NavSatFix> pending_gps_{};
  rclcpp::Time last_gps_publish_{0, 0, RCL_ROS_TIME};

  // ---- Area boundary polling state -------------------------------------------

  static constexpr double kAreaPollIntervalS = 10.0;
  static constexpr uint32_t kMaxAreaPollCount = 100;  // matches <prefix>/areas' own cap (PR #638)

  rclcpp::Time last_area_poll_{0, 0, RCL_ROS_TIME};
  bool area_poll_in_progress_{false};
  std::string last_area_boundary_json_{};
};

}  // namespace mowgli_monitoring

#endif  // MOWGLI_MONITORING__MQTT_BRIDGE_NODE_HPP_
