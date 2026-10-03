# Robot operations — start here

This directory is the short, field-tested entry point for working on the real
MowgliNext robot. It does not replace the architecture docs or source code. It
routes an operator or coding agent to the smallest authoritative reference and
records the local facts that otherwise have to be rediscovered every session.

## Fixed local facts

| Item | Value |
|---|---|
| Development machine | the small Mac, with Docker Desktop |
| Repository | `/Users/lasse/Documents/test/mowglinext-clean` |
| Robot | `mower@192.168.10.95` |
| Robot GUI | `http://192.168.10.95:4006` |
| RTK base station | `paldalen@192.168.10.99` |
| ROS container | `mowgli-ros2` (Compose service `mowgli`) |
| GUI container | `mowgli-gui` (Compose service `gui`) |
| Foxglove bridge | port `8765`, normally inside the robot stack |
| ROS distribution in the deployed image | inspect and source `/opt/ros/*/setup.bash`; the current stack uses Kilted, but one field image used a distro directory named `lyrical` |

Do not guess a username from an IP address: `.95` is `mower`; `.99` is
`paldalen`. Do not put passwords or private keys in this repository.

## Route by task

| Need | Read first |
|---|---|
| Connect, inspect containers, stop safely, test motion, diagnose Wi-Fi/power | [live-robot.md](live-robot.md) |
| Build, choose a delta, deploy one component, roll back | [build-deploy.md](build-deploy.md) |
| Find the right code/doc quickly and leave reusable evidence | [evidence-workflow.md](evidence-workflow.md) |
| GUI backend/frontend code map | [`../claude/codemaps/gui_backend.md`](../claude/codemaps/gui_backend.md), [`../claude/codemaps/gui_frontend.md`](../claude/codemaps/gui_frontend.md) |
| ROS package/code map | [`../claude/doc-index.md`](../claude/doc-index.md) |
| Topics, services, actions and TF names | [`../claude/ros-interfaces.md`](../claude/ros-interfaces.md) |
| High-level commands, manual mowing and stop semantics | [`../claude/high-level-api.md`](../claude/high-level-api.md) |
| Real motion/tuning session capture | [`../claude/session-monitoring.md`](../claude/session-monitoring.md) |
| What happened on 2026-10-02 | [`../field-reports/2026-10-02-bruno-gui-network-manual-mow.md`](../field-reports/2026-10-02-bruno-gui-network-manual-mow.md) |

## The efficient default workflow

1. Read root [`../../CLAUDE.md`](../../CLAUDE.md) safety/invariants and the
   relevant codemap. Do not grep the whole tree before using the index.
2. Run `git status --short`; preserve unrelated and pre-existing changes.
3. Inspect the live container image, immutable image ID, restart policy and
   Compose file chain before changing the robot.
4. Reproduce and capture evidence at the narrowest useful boundary.
5. Make the smallest change and add a focused regression test.
6. Build and test on the Mac. For ROS, build the full image only when the
   changed artifact requires it; otherwise use the documented script delta.
7. Create a rollback tag from the live immutable image ID.
8. Deploy only the affected Compose service.
9. Verify the actual symptom, logs, image/checksum, restart policy and resource
   use. Motion tests also require a local fallback STOP.
10. Update a current runbook for reusable procedure and a dated field report
    for observations specific to that day.

## Safety boundary

The robot has a real blade. STM32 firmware remains the sole blade/e-stop safety
authority. Never bypass its checks, and never infer that an HTTP response means
the machine is physically safe. Before recreating the ROS container or doing a
field test, prove that the robot is stationary and read live hardware status.
After a test, issue high-level `COMMAND_STOP` (8) and verify all of:

- high-level state `IDLE`;
- `mow_enabled: false`;
- `mower_esc_status: 0` / ESC power off;
- `mower_motor_rpm: 0`.

Directly requesting `mow_enabled=0` may be overwritten while the behavior tree
is still in manual mowing. `COMMAND_STOP` is the persistent operator stop path.

## Current field-proven changes

- GUI ROS state survives the final downstream unsubscribe and is replayed to a
  new browser subscriber.
- The multiplexed browser socket detects a stale live stream and reconnects.
- The manual-mow relay repeats a fresh joystick command locally, holds full
  value through 200 ms, ramps linearly to zero at the unchanged 350 ms hard
  limit, and clears it on mode exit.
- A 15 s manual-mow forward run at 0.08 m/s was judged smooth by the operator;
  explicit and fallback STOP left the blade at 0 rpm and the robot in `IDLE`.

These are observations about the 2026-10-02 deployment, not timeless source
truth. Inspect the live image before relying on them.
