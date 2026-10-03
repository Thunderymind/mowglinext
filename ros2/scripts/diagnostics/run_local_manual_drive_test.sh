#!/bin/bash
set -euo pipefail

container="mowgli-ros2"
linear="0.08"
angular="0.0"
duration="15"
armed=0
preflight_only=0

usage() {
  echo "Usage: $0 [--preflight-only | --armed] [--linear MPS] [--angular RAD_S] [--duration SEC]"
}

while (($#)); do
  case "$1" in
    --armed) armed=1; shift ;;
    --preflight-only) preflight_only=1; shift ;;
    --linear) linear="$2"; shift 2 ;;
    --angular) angular="$2"; shift 2 ;;
    --duration) duration="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

number_re='^-?[0-9]+([.][0-9]+)?$'
[[ "$linear" =~ $number_re ]] || { echo "REFUSED: invalid linear speed" >&2; exit 2; }
[[ "$angular" =~ $number_re ]] || { echo "REFUSED: invalid angular speed" >&2; exit 2; }
[[ "$duration" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "REFUSED: invalid duration" >&2; exit 2; }
awk -v x="$linear" 'BEGIN { exit !(x >= -0.20 && x <= 0.20) }' || { echo "REFUSED: linear speed exceeds 0.20 m/s" >&2; exit 2; }
awk -v x="$angular" 'BEGIN { exit !(x >= -1.0 && x <= 1.0) }' || { echo "REFUSED: angular speed exceeds 1.0 rad/s" >&2; exit 2; }
awk -v x="$duration" 'BEGIN { exit !(x > 0.0 && x <= 30.0) }' || { echo "REFUSED: duration must be in (0, 30] seconds" >&2; exit 2; }
fallback_delay=$(awk -v x="$duration" 'BEGIN { print int(x + 9.0) }')

ros() {
  docker exec "$container" bash -lc "source /ros2_ws/install/setup.bash; $1"
}

require_line() {
  local text="$1" pattern="$2" message="$3"
  if ! grep -Eq "$pattern" <<<"$text"; then
    echo "REFUSED: $message" >&2
    exit 1
  fi
}

status=$(ros "timeout 5 ros2 topic echo /behavior_tree_node/high_level_status --once")
hardware=$(ros "timeout 5 ros2 topic echo /hardware_bridge/status --once")
emergency=$(ros "timeout 5 ros2 topic echo /hardware_bridge/emergency --once")
boundary=$(ros "timeout 5 ros2 topic echo /map_server_node/boundary_violation --once")

require_line "$status" '^state_name: IDLE$' "high-level state is not IDLE"
require_line "$status" '^is_charging: false$' "robot reports charging; move it clear of the dock"
require_line "$hardware" '^esc_power: false$' "ESC power is on"
require_line "$hardware" '^mow_enabled: false$' "mowing is enabled"
require_line "$hardware" '^mower_motor_rpm: 0(\.0)?$' "blade RPM is not zero"
require_line "$emergency" '^active_emergency: false$' "active emergency"
require_line "$emergency" '^latched_emergency: false$' "latched emergency"
require_line "$boundary" '^data: false$' "boundary violation is active"

echo "Preflight OK: IDLE, undocked, blade/ESC off, no emergency or boundary violation."
if ((preflight_only)); then
  exit 0
fi
if ((!armed)); then
  echo "REFUSED: pass --armed only after confirming the robot is clear and on the mowing surface" >&2
  exit 1
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
client_source="$script_dir/local_manual_drive_client.py"
test_stamp=$(date -u +%Y%m%dT%H%M%SZ)
session="local-manual-${test_stamp}"
evidence_dir="/home/mower/hil-evidence/local-manual-drive"
mkdir -p "$evidence_dir"
docker cp "$client_source" "$container:/tmp/local_manual_drive_client.py"

stop_robot() {
  ros "timeout 6 ros2 service call /behavior_tree_node/high_level_control mowgli_interfaces/srv/HighLevelControl '{command: 8}'" >/dev/null 2>&1 || true
}

monitor_pid=""
fallback_pid=""
normal_exit=0
cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  if ((normal_exit)); then
    ros "timeout 6 ros2 service call /behavior_tree_node/high_level_control mowgli_interfaces/srv/HighLevelControl '{command: 6}'" >/dev/null 2>&1 || true
  fi
  stop_robot
  if [[ -n "$fallback_pid" ]]; then kill "$fallback_pid" >/dev/null 2>&1 || true; fi
  if [[ -n "$monitor_pid" ]]; then
    docker exec "$container" kill -INT "$monitor_pid" >/dev/null 2>&1 || true
  fi
  sleep 1
  docker cp "$container:/ros2_ws/maps/${session}.jsonl" "$evidence_dir/${session}-telemetry.jsonl" >/dev/null 2>&1 || true
  echo "Safety cleanup complete; COMMAND_STOP sent. Session: $session"
  exit "$exit_code"
}
trap cleanup EXIT INT TERM

# Independent robot-host fallback survives failure of the client process.
( sleep "$fallback_delay"; stop_robot ) >"$evidence_dir/${session}-fallback.log" 2>&1 &
fallback_pid=$!

ros "timeout 6 ros2 service call /behavior_tree_node/high_level_control mowgli_interfaces/srv/HighLevelControl '{command: 3}'"
for _ in {1..10}; do
  status=$(ros "timeout 3 ros2 topic echo /behavior_tree_node/high_level_status --once" || true)
  grep -q '^state_name: RECORDING$' <<<"$status" && break
  sleep 0.5
done
require_line "$status" '^state_name: RECORDING$' "failed to enter blade-off RECORDING mode"

hardware=$(ros "timeout 3 ros2 topic echo /hardware_bridge/status --once")
require_line "$hardware" '^mow_enabled: false$' "blade became enabled after entering RECORDING"
require_line "$hardware" '^mower_motor_rpm: 0(\.0)?$' "blade RPM became non-zero"

docker exec -d "$container" bash -lc "source /ros2_ws/install/setup.bash; exec python3 /ros2_ws/scripts/mow_session_monitor.py --session '$session' --output-dir /ros2_ws/maps"
sleep 1
monitor_pid=$(docker exec "$container" pgrep -f "mow_session_monitor.py.*${session}" | head -1)

echo "Running local-only command: linear=$linear angular=$angular duration=${duration}s"
docker exec "$container" python3 /tmp/local_manual_drive_client.py \
  --linear "$linear" --angular "$angular" --duration "$duration" \
  | tee "$evidence_dir/${session}-client.jsonl"

normal_exit=1
