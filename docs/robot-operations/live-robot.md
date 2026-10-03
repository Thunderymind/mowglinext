# Live robot operations and diagnosis

Commands here are intentionally read-only unless marked **MUTATING** or
**MOTION**. The robot is a physical machine; successful network calls are not
proof of safe mechanical state.

## Fast reachability and runtime inventory

```bash
ping -c 3 192.168.10.95
ssh -o BatchMode=yes -o ConnectTimeout=8 mower@192.168.10.95 \
  'hostname; uptime; docker ps --format "table {{.Names}}\t{{.Image}}\t{{.Status}}"'
curl --max-time 5 -I http://192.168.10.95:4006/
```

Base station:

```bash
ssh -o BatchMode=yes -o ConnectTimeout=8 paldalen@192.168.10.99 \
  'hostname; uptime'
```

If the Mac cannot reach `.95`, test from `.99`. This distinguishes a Mac/AP
path problem from the robot actually disappearing from the LAN.

## Source ROS correctly inside the container

Do not hardcode a remembered distro directory without checking the image:

```bash
ssh mower@192.168.10.95 '
  docker exec mowgli-ros2 bash -lc '\''
    ls -d /opt/ros/*
    source "$(find /opt/ros -mindepth 2 -maxdepth 2 -name setup.bash | head -n1)"
    source /ros2_ws/install/setup.bash
    ros2 node list
  '\''
'
```

For repeated work, resolve the actual setup path once, display it in the log,
then use that explicit path for the rest of the session.

## Read live safety state

```bash
ssh mower@192.168.10.95 "docker exec mowgli-ros2 bash -lc '
  source /opt/ros/kilted/setup.bash
  source /ros2_ws/install/setup.bash
  timeout 5s ros2 topic echo --once /hardware_bridge/status
  timeout 5s ros2 topic echo --once /behavior_tree_node/high_level_status
'"
```

If that exact distro path is absent, use the inspected path described above.
Important fields are high-level state/name, emergency, charging, ESC power,
`mow_enabled`, `mower_esc_status`, blade direction and motor rpm.

## Persistent stop — MUTATING

High-level command values are defined in
[`../claude/high-level-api.md`](../claude/high-level-api.md). `COMMAND_STOP` is
8 and means stop in place, blade off, remain there. `COMMAND_HOME` is 2 and
drives to the dock; do not confuse them.

```bash
curl --max-time 8 -fsS -X POST \
  -H 'Content-Type: application/json' \
  -d '{"Command":8}' \
  http://192.168.10.95:4006/api/mowglinext/call/high_level_control
```

Always follow this with live ROS status. A direct mower-disable service call
can be overwritten by the behavior tree while command 7 remains active;
high-level STOP selects the correct persistent BT branch.

## Manual mowing command chain

```text
browser joystick WebSocket
  → GUI `/api/mowglinext/publish/joy`
  → `cmd_vel_ws_relay`
  → `/cmd_vel_teleop` (twist_mux priority 20)
  → `/cmd_vel`
  → `hardware_bridge`
  → STM32
```

`COMMAND_MANUAL_MOW` is 7. The BT changes to `MANUAL_MOWING` and enables the
blade after publishing that mode. The GUI must not race it with a separate
blade-enable call. Exit with command 8.

The deployed relay design is deliberately bounded:

- browser commands normally arrive every 100 ms;
- relay timer publishes every 100 ms;
- latest non-zero command is repeated only while no older than 350 ms;
- stale input becomes zero and mode exit clears the saved command;
- if ROS/Pi/USB fails completely, the independent STM32 command watchdog still
  stops the drive after about 200 ms without packets.

The dedicated joystick WebSocket also has two heartbeat layers:

- WebSocket protocol ping every 1 s with a 3 s read deadline detects a
  half-open Wi-Fi connection and lets the frontend reconnect;
- an application ping/pong every 1 s measures browser-to-GUI round-trip time;
- the joystick shows green below 250 ms, yellow from 250 ms, and red from
  1000 ms or when no pong has arrived for 3 s;
- after 3 s without an application pong, the frontend sets the joy URL to
  `null` and restores it 50 ms later. This deliberately destroys the old
  browser WebSocket and creates a new one; merely showing `Link lost` or
  relying on protocol ping was insufficient in the observed half-open case;
- if the joystick socket is lost while the high-level state is
  `MANUAL_MOWING`, the GUI backend sends `COMMAND_STOP=8`. Reconnection does
  not silently resume a stopped manual-mow session; the operator starts it
  deliberately again. Recording mode is not stopped by this deadman.

Read-only heartbeat probe (does not publish a twist):

```bash
node /private/tmp/mowgli-joy-heartbeat-probe.mjs
```

On 2026-10-03, five fresh live connections all received pong with measured
RTT of 2–29 ms.

Foxglove subscribe/unsubscribe calls must also never run while holding the
RosProvider state mutex. A stalled network operation previously blocked cache
access and unrelated stream setup, allowing a newly reconnected joy handler to
stall before its heartbeat loop. Subscription reconciliation now serializes
Foxglove network operations under a separate mutex and rechecks desired state
after each operation.

## Controlled forward test — MOTION

Only run with explicit operator permission, robot off the dock, clear mowing
area, someone watching, and no person near the blade. Use a low speed such as
0.08 m/s. Every test needs:

1. live status showing no emergency and not charging;
2. simultaneous capture of `/cmd_vel_teleop`, `/cmd_vel_emergency` and
   `/cmd_vel`;
3. robot-local delayed command 8 as fallback;
4. explicit zero frames followed by command 8;
5. final hardware/high-level verification.

For full mowing/tuning sessions also run the persistent JSONL monitor described
in [`../claude/session-monitoring.md`](../claude/session-monitoring.md).

Field result on 2026-10-02: a 15 s forward manual-mow test at 0.08 m/s sent 60
browser frames at 250 ms spacing. The operator described motion as perfect;
final state was `IDLE`, ESC/blade off and blade rpm 0. A concurrent CLI capture
did not record the expected teleop samples even though physical motion was
seen, so that particular capture mechanism must not be treated as proof that
the relay was idle. Prefer the persistent session monitor or an unbuffered
subscriber for the next quantitative test.

### Local-only manual command isolation

Use this to distinguish browser/Wi-Fi delivery faults from Pi/USB/STM32 or
drivetrain faults. The command client runs inside `mowgli-ros2` and connects
directly to the relay on `ws://127.0.0.1:8766`; Wi-Fi is not in the command
path. The wrapper uses blade-off `RECORDING`, captures a session timeline,
sends zero frames, cancels the recording, and always issues high-level STOP.

Install the two scripts from `ros2/scripts/diagnostics/` together on the robot.
The prepared field copy is:

```text
/home/mower/hil-evidence/tools/local-manual-drive/
```

While docked, only run the non-moving gate check. A refusal because charging
is true is the expected result:

```bash
./run_local_manual_drive_test.sh --preflight-only
```

After the operator has moved the robot clear of the dock and confirmed that it
is on the mowing surface, an explicitly armed straight test is:

```bash
./run_local_manual_drive_test.sh --armed --linear 0.08 --angular 0 --duration 15
```

The wrapper refuses unless the robot is `IDLE`, not charging, blade/ESC off,
blade rpm zero, no emergency, and no boundary violation. It limits duration to
30 seconds, linear speed to 0.20 m/s and angular speed to 1.0 rad/s. Evidence
is retained in `/home/mower/hil-evidence/local-manual-drive/`; the full monitor
JSONL is copied out of the container during cleanup.

## GUI map and reconnect checks

The important logical GUI state topics are `path`, `plan`, `map`,
`mowProgress`, `lidarMap`, `robotDescription` and
`coverageResumeAvailable`. Backend mapping starts in
`gui/pkg/providers/ros.go`; frontend multiplexing is in
`gui/web/src/hooks/multiplexedSocket.ts`; map subscriptions are in
`gui/web/src/pages/map/hooks/useMapStreams.ts`.

Reload verification means more than HTTP 200:

1. start coverage so plan and mow progress exist;
2. verify both layers visually;
3. reload 5–10 times;
4. confirm both return without asking ROS to regenerate them;
5. interrupt the browser network briefly and confirm pose/live data recover;
6. inspect `mowgli-gui` and `mowgli-ros2` logs for reconnect loops/errors.

The backend intentionally retains the most recent message after the last GUI
subscriber leaves. Deleting that cache makes reload depend on Foxglove
replaying ROS2 transient-local history, which has been unreliable in practice.

## “Ping works, SSH and web do not”

Do not assume the application alone is hung. During the observed failure the
robot sometimes became unreachable even from the base station and did not
self-heal for minutes. NetworkManager logged repeated `supplicant-timeout`,
`ssid-not-found` and failed associations before DHCP eventually returned.

Collect before restarting if possible:

```bash
ssh mower@192.168.10.95 '
  date; uptime
  nmcli -f GENERAL,IP4,WIFI-PROPERTIES device show
  nmcli -f IN-USE,SSID,BSSID,CHAN,RATE,SIGNAL,SECURITY device wifi list
  journalctl -b -u NetworkManager --since "30 min ago" --no-pager
  systemctl status mowgli-network-watchdog.service --no-pager
  journalctl -b -u mowgli-network-watchdog.service --since "30 min ago" --no-pager
  ss -lntp
'
```

The watchdog needed persistent systemd directory declarations to survive
boot:

```ini
[Service]
RuntimeDirectory=mowgli-network-watchdog
RuntimeDirectoryMode=0755
StateDirectory=mowgli-network-watchdog
StateDirectoryMode=0755
```

Multiple Wi-Fi devices seen in older logs came from an earlier two-adapter
test; do not diagnose “duplicate clients” from those entries without current
interface/BSSID evidence.

## Power and USB evidence

Network loss and blade dropout overlapped recorded Pi undervoltage events.
Treat power integrity as a separate physical problem; do not mask it by
lengthening software watchdogs.

```bash
ssh mower@192.168.10.95 '
  vcgencmd get_throttled || true
  dmesg -T | grep -Ei "under.?voltage|voltage normal|usb|reset|disconnect|ttyACM"
  journalctl -b -k --no-pager | grep -Ei "under.?voltage|usb|reset|disconnect|ttyACM"
'
```

`get_throttled=0x50000` means undervoltage/throttling occurred historically;
it does not prove the fault is active at the instant read. Likewise,
`reset_cause_name=BOR` is the STM32's sticky last boot cause. Repeated status
messages with BOR do not prove repeated in-session resets. Correlate with a
boot counter/identifier, USB disconnect/re-enumeration and timestamps.

Physical checks under drive and blade load remain necessary: Pi 5 V at the
board, main battery/24 V bus, DC/DC input/output, common ground, connectors and
cable gauge. LoRa crashes may share a USB/power cause; correlate its failure
timestamp with kernel USB and undervoltage logs before changing its software.
