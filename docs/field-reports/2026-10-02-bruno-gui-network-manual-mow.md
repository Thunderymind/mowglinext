# MowgliNext field report for Bruno — GUI recovery, networking and manual mowing

Date: 2026-10-02
Robot: `mower@192.168.10.95`
Base station: `paldalen@192.168.10.99`

## Executive summary

The GUI map reload issue is substantially improved: cached ROS state now
survives the last GUI unsubscribe, and the shared frontend WebSocket detects a
half-open connection and reconnects. Planned coverage and mowing progress have
remained visible through repeated reloads in field use.

Manual mowing still exposed a separate command-stream timing defect. The
browser transmits at 10 Hz, while the STM32 stops the drive after more than
200 ms without `cmd_vel`. The relay's 350 ms grace window did not actually
repeat the latest movement command, so a small browser/Wi-Fi scheduling gap
could still trip the firmware watchdog. A minimal local fix now repeats the
last command at 10 Hz while it is at most 350 ms old, then sends zero. It also
discards the saved command on exit from manual mode. This fix passed its
focused tests and was deployed after the robot had been stopped with the blade
verified off.

There is also an independent power-integrity problem. The Pi records repeated
undervoltage and historical throttling, while the blade controller has
temporarily reported inactive/low RPM despite software continuously requesting
the blade. This must be measured under load; it must not be hidden with longer
software timeouts.

## Changes completed and deployed

### ROS state cache

- `gui/pkg/providers/ros.go`
  - Removed deletion of `lastMessage` when the final downstream subscriber
    disconnects.
  - A new GUI subscriber can therefore receive the most recent state
    immediately after reload, without waiting for Foxglove/ROS to republish
    transient-local history.
- `gui/pkg/providers/ros_provider_cache_test.go`
  - Verifies that cached state survives final unsubscribe and is replayed to a
    new subscriber.

The cache is intentionally retained for all current logical topics. The
high-value state topics are `path`, `plan`, `map`, `mowProgress`, `lidarMap`,
`robotDescription`, and `coverageResumeAvailable`. No stale-data exception was
added because deleting the cache recreated the observed reload failure, and
live topics overwrite their cache as soon as frames resume.

### Multiplexed frontend WebSocket recovery

- `gui/web/src/hooks/multiplexedSocket.ts`
  - Added a 30-second stale timeout for continuously publishing topics.
  - A stale socket is detached and closed; the existing reconnect backoff then
    creates a new connection.
  - Latched-only subscriptions such as `path` do not trigger needless churn.
  - Old socket generations can no longer change the state of a replacement
    socket.
- `gui/web/src/hooks/multiplexedSocket.test.ts`
  - Covers stale live-topic recovery and the latched-only exception.

Deployed GUI image: `mowglinext-gui:ws-stale-20261002`.

### First manual-mowing mitigation

- `ros2/src/mowgli_bringup/scripts/cmd_vel_ws_relay.py`
  - Increased the browser-command freshness window from 150 ms to 350 ms.
- `ros2/src/mowgli_bringup/test/test_manual_mow_relay.py`
- `ros2/src/mowgli_bringup/CMakeLists.txt`
  - Added the focused relay test to the package test suite.

Deployed ROS image: `mowgli-ros2:manual-smooth-20261002`
Image ID: `sha256:af37f11b8cbddc70e5f6ea913c73ecf25808f9943d2324d2eeac7bfd7519656a`
Rollback tag: `mowgli-ros2:rollback-pre-manual-smooth-20261002`

This first mitigation alone was insufficient, because it delayed zero
injection but did not keep the last non-zero command alive before the STM32's
200 ms watchdog expired.

## Latest manual-stream fix — tested and deployed

`cmd_vel_ws_relay.py` now stores the latest valid `TwistStamped`. Its existing
100 ms timer:

1. republishes that command while its browser age is at most 350 ms;
2. publishes a true zero after 350 ms;
3. clears the saved command immediately when leaving `MANUAL_MOWING`.

This preserves the firmware safety boundary:

- Browser or Wi-Fi loss: stale movement continues for no more than 350 ms,
  then becomes zero.
- Relay, Pi, ROS, or USB failure: the STM32 still receives nothing and performs
  its independent hard stop after 200 ms.
- A previous manual command cannot be resurrected in a later manual session.

Focused Docker/ROS test result: `3 passed`.

Deployment was initially postponed while the robot was mowing. After an
operator-requested stop and three consecutive status samples showing blade off
and 0 rpm, a script-only delta was built and only the `mowgli` service was
recreated.

Deployed image: `mowgli-ros2:manual-stream-20261002`
Image ID: `sha256:c612be4265e634f9c7f0ed07f254099f0feef1ca1db7ee9b44b61b11eabf17d8`
Rollback tag: `mowgli-ros2:rollback-pre-manual-stream-20261002`

Post-deploy verification confirmed the installed script checksum, relay
process/listener, GUI HTTP 200, blade off at 0 rpm, path delivery, and
mow-progress delivery. The subsequent physical manual-driving test passed; see
the field-verification section below.

## Field verification and diagnostics

### Map layers and GUI

- User confirmed that the planned pattern and mowing progress remained visible
  during mowing.
- Post-restart probes received both path and mowing-progress payloads; each was
  approximately 5–6 MB with roughly 1.5–1.6 s delivery latency.
- The web page felt materially more responsive after the reconnect/cache
  changes.

### Manual command path

An isolated API-controlled forward run at 0.08 m/s lasted about 5.21 seconds:

- 49 browser/API frames sent.
- 50 non-zero final `/cmd_vel` samples at 10 Hz.
- No internal zero gaps.
- Wheel-integrated distance approximately 0.413 m.

This proved that the backend-to-ROS path can be smooth under ideal traffic. A
real browser session remained jerky, consistent with the newly identified
100 ms browser / 200 ms firmware timing margin.

After deployment of `manual-stream-20261002`, a longer manual-mow field test
ran forward at 0.08 m/s for 15 seconds. The client sent 60 movement frames at
250 ms spacing, deliberately slower than the relay's 100 ms output cadence.
The operator described the motion as “perfect”, with no visible stop/start
jitter. Explicit STOP and the robot-local fallback STOP left the high-level
state at `IDLE`, `mow_enabled=false`, ESC off, and blade speed at 0 rpm.

A simultaneous ad-hoc `ros2 topic echo` capture unexpectedly recorded no
teleop samples and only zero values after the mux despite the observed physical
motion. The physical result and final safety status are valid; that particular
buffered capture is not suitable as quantitative evidence. A future run should
use the persistent session monitor or an explicitly unbuffered subscriber.

### Blade and power

During manual mowing, behavior and bridge logs continued requesting
`mow_enabled=true` at about 10 Hz. At the same time, map-server diagnostics
reported, in order:

- blade controller inactive;
- blade RPM below threshold;
- verified blade activity restored.

The kernel recorded repeated `Undervoltage detected!` / `Voltage normalised`
events over the same field-test period. One event lasted about 69 seconds and
several more followed. `vcgencmd get_throttled` returned `0x50000`, meaning
undervoltage and throttling have occurred.

The status field `reset_cause_name=BOR` is **not** evidence of a new STM32
reset during every manual run. Firmware captures the boot reset cause once and
repeats it in every status packet. No matching recent USB disconnect or
re-enumeration was observed, so a manual-session STM32 reboot remains
unproven.

Recommended physical checks:

1. Pi 5 V measured at the board under drive and blade load.
2. Main battery/24 V bus during blade start and dropout.
3. DC/DC input and output under load.
4. Shared ground, connectors, cable gauge, and ESC supply.

### Network recovery

The robot sometimes answered ICMP ping while SSH and the web server remained
unreachable and did not self-heal. A custom
`mowgli-network-watchdog.service` existed but failed after reboot with
`status=226/NAMESPACE`, because `ReadWritePaths` referenced runtime/state
directories before they existed.

The live robot now has this persistent systemd drop-in:

```ini
[Service]
RuntimeDirectory=mowgli-network-watchdog
RuntimeDirectoryMode=0755
StateDirectory=mowgli-network-watchdog
StateDirectoryMode=0755
```

The service was verified active after the fix, with the runtime directory
present and periodic snapshots being logged. Its accumulated statistics at the
time of inspection were 789 interruptions, 21 confirmed outages, 26 reconnect
attempts, 20 successes, 6 failures, and a longest outage of 583 seconds.

The running kernel is already `6.18.34+rpt-rpi-2712`; this is not simply a case
of still running the previously discussed `.18` kernel.

## Saved deployment procedures

- `gui/DELTA_DEPLOY.md`: backend-only and frontend-only GUI delta images.
- `ros2/DELTA_DEPLOY.md`: script-only ROS delta image, idle safety gate,
  rollback, Compose activation, and verification.

## Current state and next actions

1. Confirm quantitatively, with the persistent session monitor or an
   unbuffered subscriber, that `/cmd_vel` has no internal zero gaps during
   browser jitter; the physical 15-second test already passed visually.
2. Confirm that releasing the joystick and losing the browser both stop motion
   within the 350 ms design window.
3. Investigate the 5 V/DC/DC/ground path under load before treating the blade
   dropout as solved.
4. Add a real boot counter or boot identifier before using repeated `BOR`
   status values as evidence of an in-session STM32 reset.

## 2026-10-03 joystick liveness and latency follow-up

The dedicated joystick WebSocket now has a 1 s protocol ping, a 3 s pong
deadline, and a 1 s application heartbeat used to display round-trip latency
above the joystick. The indicator is green below 250 ms, yellow at 250–999 ms,
and red at 1000 ms or after 3 s without a heartbeat response.

Socket loss while the cached high-level state is `MANUAL_MOWING` now makes the
GUI backend send high-level `COMMAND_STOP=8`. This stops both drive and blade
instead of allowing a half-open browser connection to leave the manual-mow
state active. A reconnected socket does not resume motion automatically; the
operator must start manual mowing again. Disconnecting during `RECORDING` does
not send STOP.

Focused verification before deployment:

- Go package tests in `gui/pkg/api`: passed, including heartbeat echo,
  manual-mow disconnect STOP, and recording disconnect no-STOP.
- Frontend Vitest: 2 files / 6 tests passed, including latency and link-lost
  rendering.
- Frontend production build: passed.
- Full ARM64 GUI production image: passed.

Only GUI was recreated. The deployed thin image is
`mowglinext-gui:joy-heartbeat-20261003`, image ID
`sha256:b1163ab1c5831b2847977909d0efa72942ccbd428bea10187bebe3eeb50ee67f`.
Rollback is `mowglinext-gui:rollback-pre-joy-heartbeat-20261003`, preserving
image ID `sha256:809c510df55a274f8d05e00187075c6382cd61e08cd2ca2f66a7a13fc58f3700`.
The transfer was a 22 MB compressed delta containing only `/app/mowglinext`
and `/app/web`; the base image and all large tooling layers were reused.

Post-deploy evidence:

- HTTP 200 in 15 ms and served `index.html` checksum matched the tested image.
- Five new joystick WebSocket connections all returned heartbeat pong, with
  RTT 2–29 ms and no motion commands.
- Five independent map subscriptions replayed the 5.8 MB mowing-progress
  state, at 1.5–1.8 s. `path` timed out because the docked robot was `IDLE` and
  had no active path publisher.
- Container is running the candidate with restart policy `unless-stopped`;
  Foxglove and the command relay connected without new startup errors.
- Robot remained docked and charging in `IDLE`, with ESC off, mowing disabled,
  and blade at 0 rpm. No motion test was attempted in the dock.

During the final check, the BT temporarily remained in `EMERGENCY` and looped
`StopMoving` / `ResetEmergency` even though the aggregate status showed
`emergency=false`. The separate STM32 emergency stream had not recovered
normally. Restarting the STM32 cleared the condition. Verification afterwards
showed `IDLE`, `active_emergency=false`, `latched_emergency=false`, ESC off,
mowing disabled, and blade 0 rpm; `reset_cause_name=SFTRST` confirmed the
software reset. This was independent of the GUI heartbeat deployment: none of
the idle heartbeat probes invoked the manual-mow disconnect STOP path.

### Link-lost recovery correction

A real browser later displayed `Link lost` indefinitely. Evidence showed the
first deadman worked: at 07:57:09 the backend detected the lost joy socket and
sent `COMMAND_STOP=8`. The browser subsequently entered manual mowing again,
but its replacement socket became half-open while the application heartbeat
remained stale. Protocol ping/pong alone did not force a browser reconnect.
The robot was found in `MANUAL_MOWING` with the blade at approximately 3482
rpm; the GUI service call path timed out, so STOP was sent directly through
the ROS service. Final verification was `IDLE`, ESC off, mowing disabled, and
blade 0 rpm.

The correction has two parts:

- after 3 s without application pong, `useWS` now destroys the dedicated joy
  socket by rendering its URL as `null`, waits 50 ms, then creates a fresh
  socket at the same URL;
- RosProvider no longer holds its global state mutex across potentially
  blocking Foxglove subscribe/unsubscribe calls. Those operations use a
  separate serialization lock and reconcile against the latest listener
  state after each call.

A focused fake-timer hook test verifies the required `joy URL -> null -> joy
URL` sequence. Final test totals were Go provider/API packages passing and
three frontend test files / seven tests passing. The ARM64 production image
also built successfully.

The final deployed image is `mowglinext-gui:joy-recycle2-20261003`, image ID
`sha256:c90199b144c93c720450efcbe4d3739de1e06583a1a12e0aafdd784c85633663`.
The immediately preceding backend-fix image remains available as
`mowglinext-gui:rollback-pre-joy-recycle2-20261003`, image ID
`sha256:701dc5e3aeb2efec7480bccf80d218daff18fd32c9fecaa5f1dc8641009d2546`.
The final frontend-only transfer was 2.7 MB. Post-deploy HTTP returned 200 in
30 ms and five fresh heartbeat connections returned pong at 3–64 ms RTT.

The STM32 independently returned to the earlier emergency/status-stall state
without a manual driving test. That recurring firmware/power/USB problem is
not repaired by this GUI change and should remain a separate investigation.

### 2026-10-03 controlled left-circle test

With explicit operator authorization, the robot was commanded in `RECORDING`
mode (blade and ESC off) at constant `linear.x=0.12 m/s` and
`angular.z=+0.24 rad/s`, giving a nominal left-turn radius of 0.50 m. The
planned five laps required about 131 seconds. A robot-local 150-second STOP
fallback and a 10 Hz session monitor were armed before motion.

The test ended early when the joystick WebSocket failed after about 116
seconds. The robot travelled 13.10 m and accumulated 1599.6 degrees of wheel
yaw, about 4.44 turns. The application sent 1139 motion frames; its largest
local send interval was 103 ms and there were no local intervals over 150 ms.
The recorded `/cmd_vel` stayed exactly at 0.12/0.24 throughout the active
interval, and the monitor itself had no sample gap over 125 ms.

The motion was nevertheless not continuous. Complete wheel stops while the
command remained non-zero included:

- 1.20 s at test time 16.1 s;
- two 0.70 s stops between 26.7 and 28.1 s;
- 1.20, 0.60, 0.89 and 0.90 s stops between 46.5 and 50.6 s;
- 1.40 s at 77.4 s;
- 0.70 s at 102.3 s;
- the final stop beginning at 113.3 s, before the browser reported the
  WebSocket error.

The operator's direct visual assessment was that the run contained many
noticeable jerks and small stops. This agrees with the wheel telemetry and
rules out interpreting the zero-speed intervals as encoder noise alone; the
circle-test driving performance was visibly poor despite the constant input.

These stops line up with application-heartbeat RTT spikes. RTT reached 2078
ms, with repeated spikes in the 395-1246 ms range. This confirms that the
350 ms relay deadman is safely stopping the wheels during delivery stalls;
the remaining jerk is not caused by the browser's 10 Hz timer or changing
velocity values.

The robot kernel simultaneously reported repeated undervoltage events across
the whole run, and `vcgencmd get_throttled` returned `0x50000` (historical
undervoltage and throttling). The network watchdog also logged connectivity
failures/restores, while the associated Wi-Fi RSSI was approximately -66 to
-69 dBm. Power instability is therefore a strong common-cause candidate for
the RTT stalls, although this run alone does not prove whether every short
stall originated in power or radio delivery.

After the test, a direct high-level `COMMAND_STOP=8` returned success. Final
verification showed `IDLE`, requested and applied velocity zero, blade 0 rpm,
ESC off, no emergency, and no boundary violation. Raw evidence is retained on
the robot at:

- `/home/mower/hil-evidence/2026-10-03-circle-left-r05-5laps-v1.jsonl`
- `/home/mower/hil-evidence/2026-10-03-circle-left-r05-5laps-client.jsonl`

### Robot-local stale-command ramp

As a minimal follow-up to the visibly abrupt deadman stops, the manual-command
relay now holds a fresh command at full value through 200 ms of age, then
scales every velocity axis linearly to zero at the unchanged 350 ms hard
limit. Its local publish timer is 50 ms, providing intermediate deceleration
steps while continuing to feed the firmware watchdog. Commands older than
350 ms still produce an immediate true zero; the safety window was not
extended.

The focused ROS test suite passed all four tests, covering timing constraints,
full-speed replay, the 50% ramp midpoint, stale-command zero, all six velocity
axes, and clearing saved movement on mode exit. Python compilation and
`git diff --check` also passed.

Only the installed `cmd_vel_ws_relay.py` was deployed as a Docker delta on top
of the exact active base. The active image is
`mowgli-ros2:manual-ramp-20261003`, image ID
`sha256:4c3fe6312b27528d5da4c75be2a556f465281825c08d05e55440882c07db27dc`.
Rollback is `mowgli-ros2:rollback-pre-manual-ramp-20261003`, preserving image
ID `sha256:c612be4265e634f9c7f0ed07f254099f0feef1ca1db7ee9b44b61b11eabf17d8`.
The deployed script SHA-256 is
`6c3c1c57fcc665ac4612475bab60ab42f5336f6b334dca17ea0a7020a849afe9`.

Only the ROS Compose service was recreated. Post-deploy checks showed zero
container restarts, relay listening on port 8766 with the GUI client connected,
GUI HTTP 200 in 10 ms, and a fresh joystick heartbeat RTT of 3 ms. The robot
remained `IDLE`, requested/applied velocity zero, blade 0 rpm, ESC off, with no
emergency or boundary violation. No physical drive test was performed after
this deployment.

## Scope note

The working tree contains other pre-existing changes unrelated to this field
report. They were not reverted, staged, or attributed to this work. This report
only covers the files and live-system changes explicitly listed above.
