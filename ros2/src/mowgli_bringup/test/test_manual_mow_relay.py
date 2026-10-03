import re
import importlib.util
from pathlib import Path
from unittest.mock import Mock

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[2]


def _numeric_constant(source: str, name: str) -> float:
    match = re.search(rf"^{name}\s*=\s*([0-9.]+)\s*$", source, re.MULTILINE)
    assert match, f"missing numeric constant {name}"
    return float(match.group(1))


def test_joystick_stale_window_tolerates_wifi_jitter_but_precedes_mux_timeout():
    relay_source = (PACKAGE_ROOT / "scripts/cmd_vel_ws_relay.py").read_text()
    frontend_source = (
        REPO_ROOT / "gui/web/src/pages/map/hooks/useManualMode.ts"
    ).read_text()
    mux = yaml.safe_load((PACKAGE_ROOT / "config/twist_mux.yaml").read_text())

    active_s = _numeric_constant(relay_source, "_JOYSTICK_ACTIVE_S")
    ramp_start_s = _numeric_constant(relay_source, "_JOYSTICK_RAMP_START_S")
    idle_period_s = _numeric_constant(relay_source, "_MANUAL_IDLE_PERIOD_S")
    interval_match = re.search(
        r"^const JOY_SEND_INTERVAL_MS = ([0-9]+);$", frontend_source, re.MULTILINE
    )
    assert interval_match, "missing JOY_SEND_INTERVAL_MS"
    send_interval_s = int(interval_match.group(1)) / 1000.0
    teleop_timeout_s = mux["twist_mux"]["ros__parameters"]["topics"]["teleop"][
        "timeout"
    ]

    # A held command survives two delayed/missed browser repeats at full speed,
    # then ramps down before the downstream mux's hard timeout.
    assert active_s >= 3 * send_interval_s
    assert 2 * send_interval_s <= ramp_start_s < active_s < teleop_timeout_s
    assert idle_period_s <= active_s - ramp_start_s


def _load_relay_module():
    path = PACKAGE_ROOT / "scripts/cmd_vel_ws_relay.py"
    spec = importlib.util.spec_from_file_location("cmd_vel_ws_relay", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_relay_republishes_fresh_command_ramps_then_zeroes_stale_command(monkeypatch):
    relay_module = _load_relay_module()
    relay = relay_module.CmdVelRelayNode.__new__(relay_module.CmdVelRelayNode)
    relay._manual_mowing = True
    relay._last_joystick_command = 100.0
    relay._last_joystick_twist = relay_module.TwistStamped()
    relay._last_joystick_twist.twist.linear.x = 0.12
    relay._last_joystick_twist.twist.angular.z = 0.24
    relay._pub = Mock()
    zero = relay_module.TwistStamped()
    relay._zero_twist = Mock(return_value=zero)

    monkeypatch.setattr(relay_module.time, "monotonic", lambda: 100.15)
    relay._publish_manual_command()
    relay._pub.publish.assert_called_once_with(relay._last_joystick_twist)

    relay._pub.publish.reset_mock()
    scaled = object()
    relay._scaled_twist = Mock(return_value=scaled)
    monkeypatch.setattr(relay_module.time, "monotonic", lambda: 100.275)
    relay._publish_manual_command()
    scaled_source, scaled_factor = relay._scaled_twist.call_args.args
    assert scaled_source is relay._last_joystick_twist
    assert scaled_factor == pytest.approx(0.5)
    relay._pub.publish.assert_called_once_with(scaled)

    relay._pub.publish.reset_mock()
    monkeypatch.setattr(relay_module.time, "monotonic", lambda: 100.36)
    relay._publish_manual_command()
    relay._pub.publish.assert_called_once_with(zero)


def test_scaled_twist_scales_all_velocity_axes_and_refreshes_header():
    relay_module = _load_relay_module()
    relay = relay_module.CmdVelRelayNode.__new__(relay_module.CmdVelRelayNode)
    source = relay_module.TwistStamped()
    source.twist.linear.x = 0.12
    source.twist.linear.y = -0.08
    source.twist.linear.z = 0.04
    source.twist.angular.x = -0.4
    source.twist.angular.y = 0.2
    source.twist.angular.z = 0.24
    output = relay_module.TwistStamped()
    relay._zero_twist = Mock(return_value=output)

    scaled = relay._scaled_twist(source, 0.25)

    assert scaled is output
    assert scaled.twist.linear.x == pytest.approx(0.03)
    assert scaled.twist.linear.y == pytest.approx(-0.02)
    assert scaled.twist.linear.z == pytest.approx(0.01)
    assert scaled.twist.angular.x == pytest.approx(-0.1)
    assert scaled.twist.angular.y == pytest.approx(0.05)
    assert scaled.twist.angular.z == pytest.approx(0.06)


def test_leaving_manual_mode_discards_saved_command():
    relay_module = _load_relay_module()
    relay = relay_module.CmdVelRelayNode.__new__(relay_module.CmdVelRelayNode)
    relay._last_joystick_command = 100.0
    relay._last_joystick_twist = object()
    status = Mock(state=0)

    relay._on_high_level_status(status)

    assert relay._manual_mowing is False
    assert relay._last_joystick_command == 0.0
    assert relay._last_joystick_twist is None
