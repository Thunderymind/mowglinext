# Evidence workflow — learn once, reuse cheaply

The goal is to replace rediscovery with small, trustworthy references. Save
procedures as current runbooks; save dated observations as field reports.

## Reference order

Use this order and stop as soon as the answer is supported:

1. Root `CLAUDE.md` for safety and architecture invariants.
2. `docs/claude/doc-index.md` to learn which documents are current, generated,
   historical or superseded.
3. The relevant codemap for file ownership, runtime surface, test command and
   pitfalls.
4. Generated `ros-interfaces.md`, `parameters.md` and `testing-ci.md` for exact
   names and commands.
5. Source code and focused `rg` around the named component.
6. Focused tests.
7. Immutable runtime facts from Docker inspect, ROS graph/status and logs.
8. Dated field evidence and operator observation.
9. Internet/issue search only when the answer depends on external behavior,
   kernel/driver history or a version that may have changed.

Code and live runtime beat prose when they disagree. A dated field report is
evidence about that deployment, not a permanent architecture rule.

## First five minutes of a robot task

```bash
cd /Users/lasse/Documents/test/mowglinext-clean
git status --short
git branch --show-current
git rev-parse HEAD
docker desktop status
docker system df
ssh mower@192.168.10.95 \
  'date; uptime; docker ps --format "table {{.Names}}\t{{.Image}}\t{{.Status}}"'
```

Then open only the task-specific references. Examples:

- GUI reconnect/map: GUI backend codemap, GUI frontend codemap,
  `ros.go`, `multiplexedSocket.ts`, `useMapStreams.ts`.
- Manual mowing: high-level API, bringup codemap, hardware codemap,
  `cmd_vel_ws_relay.py`, `twist_mux.yaml`, BT manual/stop branches.
- Deploy: deploy codemap plus the GUI or ROS delta guide.
- Network/power: live-robot runbook and a timestamped journal slice.

## Evidence bundle for every deployment

Record these in a dated `docs/field-reports/YYYY-MM-DD-<subject>.md`:

- repository commit/branch and whether the tree was dirty;
- exactly which files changed;
- focused test commands and results;
- local image ref, immutable ID, created time and architecture;
- old robot image ref/ID/created time and rollback tag;
- active Compose chain and service recreated;
- transferred artifact checksum;
- new live image ID, restart policy and start time;
- symptom-level verification, logs and resource comparison;
- rollback command;
- open physical or software uncertainties.

Never call a test successful merely because the command returned zero. For a
GUI issue, prove the layers/reconnect. For motion, prove physical behavior and
final safe state. For a network fault, preserve timestamps and test from the
base station as well as the Mac.

## Turn discoveries into durable references

- Reusable command or decision: update this runbook.
- New source ownership/runtime path: update the relevant codemap generator or
  maintained reference, not a random note.
- One-day observation/image tag/log: dated field report.
- New regression: focused automated test next to the component.
- Temporary probe: keep outside the repo, but copy its essential method and
  result into the field report.
- Contradictory evidence: record both sides and the limitation. Do not smooth
  it into a false conclusion.

## Token-efficient command style

- Ask `rg --files` or the codemap before broad searches.
- Query exact time windows (`journalctl --since`) and exact containers instead
  of dumping all logs.
- Use `docker inspect --format` to extract only image ID, tag, start time,
  labels and restart policy.
- Capture three adjacent boundaries when locating dropped commands: source
  topic, arbitration/mux topic and hardware-facing topic.
- Use one focused regression test before a workspace-wide suite.
- Link to authoritative long docs rather than copying them into reports.
- Give temporary artifacts unique names containing date/purpose and checksum
  anything transferred to the robot.

## End-of-session checklist

1. Robot stopped; high-level and hardware status recorded.
2. Blade verified off at 0 rpm.
3. Live image IDs and restart policies recorded.
4. Test output and field result recorded separately.
5. Rollback remains available.
6. Current runbook updated only with reusable facts.
7. Dated report updated with deployment-specific facts.
8. `git diff --check` and `git status --short` reviewed; unrelated changes left
   untouched.
