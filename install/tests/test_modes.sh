#!/usr/bin/env bash
# =============================================================================
# mowglinext.sh modes: install (default) | update | repair | check, --help,
# and the non-interactive contract. update/repair act on an EXISTING install
# and must refuse, not prompt, when there is none.
# =============================================================================
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/framework.sh
source "$SCRIPT_DIR/lib/framework.sh"
# shellcheck source=lib/mocks.sh
source "$SCRIPT_DIR/lib/mocks.sh"
# shellcheck source=lib/harness.sh
source "$SCRIPT_DIR/lib/harness.sh"

setup_sandbox
install_all_mocks

section "--help prints the mode contract without touching anything"
repo="$SANDBOX/repo_help"
sandbox_repo "$repo"
harness_init "$repo"
out="$(bash "$repo/install/mowglinext.sh" --help 2>&1)"; ec=$?
assert_eq "--help exits 0" "0" "$ec"
for word in "install" "update" "repair" "check" "--non-interactive" "--no-updater"; do
  assert_contains "--help documents $word" "$word" "$out"
done
assert_file_not_exists "--help writes no docker/.env" "$repo/docker/.env"

section "update and repair refuse a robot that was never installed"
for mode in update repair; do
  repo="$SANDBOX/repo_$mode"
  sandbox_repo "$repo"
  harness_init "$repo"
  out="$(timeout 60 bash "$repo/install/mowglinext.sh" "$mode" 2>&1)" && ec=0 || ec=$?
  assert_neq "$mode without docker/.env exits non-zero" "0" "$ec"
  assert_contains "$mode names the missing installation" "No installation found" "$out"
  assert_not_contains "$mode never asks a hardware question" "Select hardware backend" "$out"
  assert_file_not_exists "$mode writes no docker/.env" "$repo/docker/.env"
done

section "check is the --check alias and reports a missing runtime"
repo="$SANDBOX/repo_check"
sandbox_repo "$repo"
harness_init "$repo"
out="$(timeout 60 bash "$repo/install/mowglinext.sh" check 2>&1)" && ec=0 || ec=$?
assert_neq "check without a generated compose exits non-zero" "0" "$ec"
assert_contains "check points at install" "install' first" "$out"

section "non-interactive install seeds every unset choice with its default"
repo="$SANDBOX/repo_ni"
sandbox_repo "$repo"
harness_init "$repo"
harness_set_preset gnss=auto gnss_connection=uart lidar=none
if harness_run; then pass "non-interactive harness_run"; else fail "non-interactive harness_run" "non-zero exit"; fi
env_ni="$(cat "$repo/docker/.env")"
assert_contains "GNSS device defaults to the UART header port" "GNSS_SERIAL_DEVICE=/dev/ttyAMA4" "$env_ni"
assert_contains "GNSS baud defaults to the sidecar canonical value" "GNSS_SERIAL_BAUD=921600" "$env_ni"
assert_contains "LiDAR off when not requested" "LIDAR_ENABLED=false" "$env_ni"
yaml_ni="$(cat "$repo/docker/config/mowgli/mowgli_robot.yaml")"
assert_match "datum is left as the seed placeholder (GUI-owned)" '^[[:space:]]+datum_lat:[[:space:]]+0(\.0+)?[[:space:]]*$' "$yaml_ni"
assert_match "ntrip is left as the seed default (GUI-owned)" '^[[:space:]]+ntrip_enabled:[[:space:]]+false[[:space:]]*$' "$yaml_ni"
assert_match "dock pose is left as the seed value (calibration output)" '^[[:space:]]+dock_pose_x:' "$yaml_ni"

section "a bare non-interactive run on an installed robot means update, not a reinstall"
repo="$SANDBOX/repo_bare"
sandbox_repo "$repo"
harness_init "$repo"
mkdir -p "$repo/docker"; printf 'ROS_DOMAIN_ID=0\n' > "$repo/docker/.env"
out="$(timeout 60 bash "$repo/install/mowglinext.sh" --non-interactive --only=env 2>&1)" && ec=0 || ec=$?
assert_eq "explicit --only run exits 0" "0" "$ec"
assert_not_contains "no mode menu without a terminal" "What do you want to do" "$out"
out="$(timeout 120 bash "$repo/install/mowglinext.sh" --non-interactive 2>&1)" && ec=0 || ec=$?
assert_contains "bare no-tty run on an installed robot picks update" "Mode: update" "$out"
assert_not_contains "bare no-tty run does not reconfigure hardware" "Configuring GNSS serial link" "$out"
out="$(timeout 120 bash "$repo/install/mowglinext.sh" --non-interactive --lidar=none 2>&1)" && ec=0 || ec=$?
assert_not_contains "hardware flags keep a reconfiguring install" "Mode: update" "$out"

section "an existing mowgli_robot.yaml is never written, even when it is root-owned"
repo="$SANDBOX/repo_perm"
sandbox_repo "$repo"
harness_init "$repo"
harness_set_preset gnss=auto gnss_connection=uart lidar=none
harness_run >/dev/null 2>&1 || fail "seed run" "non-zero exit"
yaml="$repo/docker/config/mowgli/mowgli_robot.yaml"
printf '        use_scan_matching: true\n' >> "$yaml"   # an operator/legacy key the installer used to strip
before="$(cat "$yaml")"
chmod 0444 "$yaml"
LIDAR_ENABLED=true; LIDAR_TYPE=ldlidar
if write_config >/dev/null 2>&1; then pass "write_config on an existing unwritable yaml returns 0"; else fail "write_config on an existing unwritable yaml" "returned non-zero"; fi
chmod 0644 "$yaml"
assert_eq "existing yaml is byte-identical after write_config" "$before" "$(cat "$yaml")"
assert_not_contains "no python edit was attempted on the operator's file" "Could not update" "$(write_config 2>&1)"

section "Watchtower is gone and the MQTT broker is opt-in"
repo="$SANDBOX/repo_mqtt"
sandbox_repo "$repo"
harness_init "$repo"
harness_set_preset gnss=auto gnss_connection=uart lidar=none
harness_run >/dev/null 2>&1 || fail "default run" "non-zero exit"
compose_default="$(cat "$repo/docker/docker-compose.yaml")"
assert_not_contains "no Watchtower service in a fresh install" "mowgli-watchtower" "$compose_default"
assert_not_contains "no mosquitto by default" "mowgli-mqtt" "$compose_default"
assert_contains ".env records the broker as off" "ENABLE_MQTT=false" "$(cat "$repo/docker/.env")"
harness_set_preset mqtt=true
harness_run >/dev/null 2>&1 || fail "mqtt run" "non-zero exit"
assert_contains "mosquitto composed when requested" "mowgli-mqtt" "$(cat "$repo/docker/docker-compose.yaml")"
assert_contains ".env records the broker as on" "ENABLE_MQTT=true" "$(cat "$repo/docker/.env")"
assert_file_not_exists "watchtower fragment removed" "$REPO_ROOT/install/compose/docker-compose.watchtower.yml"
# An install that predates the toggle (.env without ENABLE_MQTT) keeps its broker.
repo="$SANDBOX/repo_mqtt_legacy"
sandbox_repo "$repo"
harness_init "$repo"
mkdir -p "$repo/docker"
printf 'services:\n  mosquitto:\n    container_name: mowgli-mqtt\n' > "$repo/docker/docker-compose.yaml"
harness_set_preset gnss=auto gnss_connection=uart lidar=none
harness_run >/dev/null 2>&1 || fail "legacy mqtt run" "non-zero exit"
assert_contains "existing broker is kept when .env has no ENABLE_MQTT" "ENABLE_MQTT=true" "$(cat "$repo/docker/.env")"
assert_contains "existing broker stays composed" "mowgli-mqtt" "$(cat "$repo/docker/docker-compose.yaml")"

section "--only= still lists the current step names"
repo="$SANDBOX/repo_only"
sandbox_repo "$repo"
harness_init "$repo"
out="$(bash "$repo/install/mowglinext.sh" --only=bogus 2>&1)" && ec=0 || ec=$?
assert_neq "--only=bogus exits non-zero" "0" "$ec"
for step in helpers mower startup; do assert_contains "--only lists $step" "$step" "$out"; done
assert_not_contains "--only no longer lists rangefinders" "rangefinders" "$out"
assert_not_contains "--only no longer lists tools" " tools " "$out"

test_summary
