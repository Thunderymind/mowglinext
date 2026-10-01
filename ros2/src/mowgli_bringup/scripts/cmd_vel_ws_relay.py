#!/usr/bin/env python3
"""
cmd_vel_ws_relay — minimal WebSocket relay for /cmd_vel_teleop.

Accepts TwistStamped JSON on port 8766 and publishes directly to
/cmd_vel_teleop via rclpy, bypassing foxglove_bridge's JSON→CDR
conversion overhead and shared-connection head-of-line blocking with
subscription data.

Wire format (client → relay, newline-delimited JSON or bare JSON frames):
  {"twist": {"linear": {"x": 0.2, "y": 0, "z": 0},
             "angular": {"x": 0, "y": 0, "z": 0.5}}}

The header field is accepted but ignored; stamp defaults to zeros, which
is fine because twist_mux only checks the arrival time of the message, not
the header stamp.
"""
import asyncio
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from mowgli_interfaces.msg import HighLevelStatus
import websockets
import websockets.exceptions

_RELAY_PORT = 8766
# Velocity clamps applied before publishing — defense-in-depth so a
# malformed or hostile frame can't command extreme motor speeds.
_MAX_LINEAR_MPS = 2.0
_MAX_ANGULAR_RAD_S = 5.0
_MANUAL_IDLE_PERIOD_S = 0.1
_JOYSTICK_ACTIVE_S = 0.15


class CmdVelRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_ws_relay")
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(TwistStamped, "/cmd_vel_teleop", qos)
        self._manual_mowing = False
        self._last_joystick_command = 0.0
        self._mode_sub = self.create_subscription(
            HighLevelStatus,
            "/behavior_tree_node/high_level_status",
            self._on_high_level_status,
            10,
        )
        self._manual_idle_timer = self.create_timer(
            _MANUAL_IDLE_PERIOD_S, self._publish_manual_idle
        )
        self.get_logger().info("cmd_vel_ws_relay: publisher ready on /cmd_vel_teleop")

    def _zero_twist(self) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_footprint"
        return msg

    def _on_high_level_status(self, msg: HighLevelStatus) -> None:
        self._manual_mowing = msg.state == HighLevelStatus.HIGH_LEVEL_STATE_MANUAL_MOWING

    def _publish_manual_idle(self) -> None:
        if not self._manual_mowing:
            return
        if time.monotonic() - self._last_joystick_command <= _JOYSTICK_ACTIVE_S:
            return

        # The STM32 deliberately requires a fresh velocity authorization before
        # accepting a blade request. Keep that authorization alive with a true
        # zero while manual mowing is selected and the joystick is at rest.
        # This timer lives in the host process, so a host/link failure still
        # stops the stream and lets the firmware timeout fail safe.
        self._pub.publish(self._zero_twist())

    def publish_json(self, raw: str) -> None:
        d = json.loads(raw)
        t = d.get("twist", {})
        lin = t.get("linear", {})
        ang = t.get("angular", {})

        def clamp(v: float, limit: float) -> float:
            return max(-limit, min(limit, v))

        msg = TwistStamped()
        msg.twist.linear.x = clamp(float(lin.get("x", 0.0)), _MAX_LINEAR_MPS)
        msg.twist.linear.y = clamp(float(lin.get("y", 0.0)), _MAX_LINEAR_MPS)
        msg.twist.linear.z = clamp(float(lin.get("z", 0.0)), _MAX_LINEAR_MPS)
        msg.twist.angular.x = clamp(float(ang.get("x", 0.0)), _MAX_ANGULAR_RAD_S)
        msg.twist.angular.y = clamp(float(ang.get("y", 0.0)), _MAX_ANGULAR_RAD_S)
        msg.twist.angular.z = clamp(float(ang.get("z", 0.0)), _MAX_ANGULAR_RAD_S)
        self._last_joystick_command = time.monotonic()
        self._pub.publish(msg)


# Module-level node shared between the asyncio loop (main thread) and the
# rclpy spin thread.
_node: CmdVelRelayNode


async def _ws_handler(websocket) -> None:
    addr = websocket.remote_address
    _node.get_logger().info(f"cmd_vel_ws_relay: client connected {addr}")
    try:
        async for raw in websocket:
            try:
                _node.publish_json(raw)
            except (ValueError, KeyError) as exc:
                _node.get_logger().warn(f"cmd_vel_ws_relay: bad message: {exc}")
    except websockets.exceptions.ConnectionClosedError:
        pass
    finally:
        _node.get_logger().info(f"cmd_vel_ws_relay: client {addr} disconnected")


async def _serve() -> None:
    # ping_interval=None disables the WebSocket keep-alive ping so the
    # relay does not send periodic frames that could delay publish writes.
    async with websockets.serve(
        _ws_handler,
        "127.0.0.1",
        _RELAY_PORT,
        ping_interval=None,
        max_size=65536,
    ):
        _node.get_logger().info(f"cmd_vel_ws_relay: listening on :{_RELAY_PORT}")
        await asyncio.Future()  # run until cancelled


def main() -> None:
    global _node
    rclpy.init()
    _node = CmdVelRelayNode()

    # rclpy.spin in a daemon thread. For a pure-publisher node the spin loop
    # has no callbacks to run — it just keeps the DDS participant alive and
    # allows graceful shutdown. Daemon=True means it exits when asyncio ends.
    spin_thread = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    spin_thread.start()

    try:
        asyncio.run(_serve())
    except KeyboardInterrupt:
        pass
    finally:
        _node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
