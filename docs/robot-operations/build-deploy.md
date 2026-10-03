# Build and deploy with the smallest safe delta

Start with the change type. The fastest path is the smallest artifact that is
both testable and compatible with the exact image already running on the
robot.

## Decision table

| Changed files | Build/deploy path | Authoritative procedure |
|---|---|---|
| Go backend only | Full GUI image test/build on Mac, extract one backend binary, thin robot-side image | [`../../gui/DELTA_DEPLOY.md`](../../gui/DELTA_DEPLOY.md) |
| `gui/web/**` only | Node build exports `dist`; thin image copies only `/app/web` | [`../../gui/DELTA_DEPLOY.md`](../../gui/DELTA_DEPLOY.md) |
| GUI assets, Dockerfile or runtime packages | Full GUI image transfer/push | GUI Dockerfile and deploy codemap |
| One installed ROS Python script | Test in ROS image, copy script into one thin layer on live base | [`../../ros2/DELTA_DEPLOY.md`](../../ros2/DELTA_DEPLOY.md) |
| ROS C++, interfaces, launch, config or dependencies | Full ARM64 ROS image | [`../claude/commands.md`](../claude/commands.md), [`../claude/codemaps/ros2_workspace_tooling.md`](../claude/codemaps/ros2_workspace_tooling.md) |
| Firmware | PlatformIO build and the firmware-specific verified flash flow | [`../claude/codemaps/firmware.md`](../claude/codemaps/firmware.md) |

Never use a script-only delta for C++, launch files, config, interfaces or
dependencies. A tiny transfer is only safe if the full runtime base remains
compatible with that artifact.

## Pre-deploy snapshot

Run this before every robot mutation and save the output in the field report:

```bash
ssh mower@192.168.10.95 '
  docker inspect mowgli-gui mowgli-ros2 \
    --format "{{.Name}} image_ref={{.Config.Image}} image_id={{.Image}} started={{.State.StartedAt}} restart={{.HostConfig.RestartPolicy.Name}}"
  docker image inspect \
    "$(docker inspect mowgli-gui --format "{{.Image}}")" \
    "$(docker inspect mowgli-ros2 --format "{{.Image}}")" \
    --format "id={{.Id}} created={{.Created}} tags={{json .RepoTags}}"
'
```

Discover the active Compose chain instead of assuming a checkout or filename:

```bash
ssh mower@192.168.10.95 '
  docker inspect mowgli-gui \
    --format "workdir={{index .Config.Labels \"com.docker.compose.project.working_dir\"}} files={{index .Config.Labels \"com.docker.compose.project.config_files\"}}"
'
```

Preserve the immutable current image ID under a new rollback tag. Never retag
or overwrite the old runtime tag with the candidate.

## Local builds on the small Mac

GUI production build, from the repo root:

```bash
cd gui
go test ./...
cd ..
docker build --platform linux/arm64 \
  -t mowglinext-gui:<tag> -f gui/Dockerfile gui
```

Frontend-only work should use the BuildKit export recipe in
[`gui/DELTA_DEPLOY.md`](../../gui/DELTA_DEPLOY.md); it avoids rebuilding and
transferring the large firmware-tooling layers.

Full ROS runtime image (context must be the repository root):

```bash
docker build --platform linux/arm64 \
  -f ros2/Dockerfile --target runtime -t mowgli-ros2:<tag> .
```

For a Python relay-only change, run focused pytest and `py_compile`, then use
the script-only delta procedure. The deployed manual-relay test is:

```bash
python3 -m pytest -q \
  ros2/src/mowgli_bringup/test/test_manual_mow_relay.py
python3 -m py_compile \
  ros2/src/mowgli_bringup/scripts/cmd_vel_ws_relay.py
git diff --check
```

The ROS Dockerfile's build stage tolerates test failure (`|| true` in the
image build). A successful image build is therefore not proof that tests
passed; run the focused tests explicitly.

## Activation rule

- GUI-only change: recreate only Compose service `gui`.
- ROS-only change: recreate only Compose service `mowgli`.
- Append the candidate override last in the exact existing Compose file order.
- Use `--no-deps --force-recreate`; do not restart the whole robot stack unless
  the change truly crosses service boundaries.
- ROS activation waits until the robot is stationary and the blade is verified
  off.
- Verify `restart: unless-stopped` after recreation.

## Minimum verification

```bash
curl --max-time 5 -fsS -o /dev/null -w '%{http_code}\n' \
  http://192.168.10.95:4006/
ssh mower@192.168.10.95 'docker ps --format "table {{.Names}}\t{{.Image}}\t{{.Status}}"'
```

Then verify the symptom, not merely container health:

- GUI: ROS data, robot pose, plan and mow progress; reload 5–10 times; brief
  network interruption; inspect GUI and Foxglove logs.
- ROS: node/topic/service relevant to the change, GUI reconnect, USB/serial
  logs, and live hardware status.
- Motion: capture command topics, arm a robot-local fallback STOP, remain in
  the mowing area, and verify final `IDLE` plus blade 0 rpm.
- Compare CPU/memory with the pre-deploy snapshot.

## Disk pressure without deleting useful work

Inventory before cleanup:

```bash
df -h /Users/lasse /private/tmp
docker system df -v
git worktree list --porcelain
du -sh worktrees/* 2>/dev/null | sort -h
find worktrees -maxdepth 4 -type d -name node_modules -prune \
  -exec du -sh {} \; 2>/dev/null | sort -h
```

Large size is commonly duplicated worktrees, `node_modules`, Docker build
cache and expanded multi-stage image layers. Do not delete a worktree until its
branch, uncommitted changes and attachment to an active task are known. Prefer
removing a proven-unused worktree or targeted build cache over broad recursive
deletion. Keep the live and rollback image IDs until field verification ends.
