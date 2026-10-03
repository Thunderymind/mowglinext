# ROS 2 script-only delta deployment

Use this procedure when a tested change only replaces an installed ROS 2
Python script. It creates one small image layer on top of the exact image
already running on the robot. Do not use it for C++, interfaces, launch files,
configuration, dependencies, or firmware.

## Safety gate

Do not recreate the ROS container while the robot is moving, mowing, docking,
or charging through a transition. Confirm an idle high-level state first. A
delta image may be built while idle, but container activation must wait until
the robot is stationary and the blade is verified off.

## 1. Test the source

Run the focused tests in a ROS image with the repository mounted read-only:

```bash
docker run --rm -v <repo>:/work:ro <local-ros-image> bash -lc '
  source /opt/ros/kilted/setup.bash
  python3 -m pytest -q /work/ros2/src/mowgli_bringup/test/test_manual_mow_relay.py
'
```

Also run `python3 -m py_compile` and `git diff --check` for the changed script
and test.

## 2. Record the exact runtime base

```bash
ssh mower@<robot-ip> '
  docker inspect mowgli-ros2 \
    --format "image={{.Config.Image}} id={{.Image}} restart={{.HostConfig.RestartPolicy.Name}} started={{.State.StartedAt}}"
  docker image inspect "$(docker inspect mowgli-ros2 --format "{{.Image}}")" \
    --format "id={{.Id}} created={{.Created}} tags={{json .RepoTags}}"
'
```

Read the active Compose file chain from the container labels. Preserve its
order and append the new override last.

## 3. Preserve rollback and build one delta layer

Tag the immutable current image ID before building:

```bash
docker tag <current-image-id> mowgli-ros2:rollback-pre-<delta-tag>
```

For `cmd_vel_ws_relay.py`, transfer only the tested script and this Dockerfile:

```dockerfile
FROM <current-image-tag>
COPY cmd_vel_ws_relay.py /ros2_ws/install/mowgli_bringup/lib/mowgli_bringup/cmd_vel_ws_relay.py
RUN chmod 0755 /ros2_ws/install/mowgli_bringup/lib/mowgli_bringup/cmd_vel_ws_relay.py
```

Build on the robot, where the exact base image already exists:

```bash
docker build -t mowgli-ros2:<delta-tag> <evidence-directory>
```

Use a new tag; never overwrite the current or rollback tag.

## 4. Activate only the ROS service

Create an override:

```yaml
services:
  mowgli:
    image: mowgli-ros2:<delta-tag>
```

Append it to the existing ordered Compose chain and recreate only `mowgli`:

```bash
docker compose -f <existing-1> -f <existing-2> -f <new-override> \
  up -d --no-deps --force-recreate mowgli
```

## 5. Verify

- Expected image ID/tag and `restart: unless-stopped`.
- ROS/Foxglove/GUI reconnect after the ROS restart.
- Hardware bridge, high-level state, pose, map, plan, and mowing progress.
- No new serial, USB, watchdog, or container restart errors.
- For motion changes, run a monitored field test only inside the mowing area.

Rollback by selecting `mowgli-ros2:rollback-pre-<delta-tag>` in a final
override and recreating only `mowgli` with the same Compose chain.
