# GUI delta deployment

Use this procedure for a fast deployment to a robot when a GUI change only
touches the Go backend or the web frontend. It reuses the robot's existing GUI
image and transfers only the changed artifact instead of the roughly 2 GB of
expanded firmware-tooling layers in the full image.

Do **not** use a backend-binary delta when any of these changed:

- `gui/web/**`
- `gui/asserts/**`
- `gui/Dockerfile`
- runtime packages or firmware tools in the image

When only `gui/web/**` changed, use the frontend-only procedure below. Transfer
or push the full image when `gui/asserts/**`, `gui/Dockerfile`, runtime packages,
or firmware tools changed.

## Backend-only delta

## 1. Test and build the full image locally

The full build is still required. It proves that the exact source tree builds
with the production Dockerfile before a binary is extracted from it.

```bash
cd <repo>/gui
go test ./...
cd ..
docker build --platform linux/arm64 \
  -t mowglinext-gui:<delta-tag> \
  -f gui/Dockerfile gui
```

Record the local image identity:

```bash
docker image inspect mowglinext-gui:<delta-tag> \
  --format 'ID={{.Id}} CREATED={{.Created}} SIZE={{.Size}} ARCH={{.Architecture}}'
```

## 2. Record and preserve the robot's rollback image

```bash
ssh mower@<robot-ip> \
  'docker inspect mowgli-gui --format "IMAGE_REF={{.Config.Image}} IMAGE_ID={{.Image}} STARTED={{.State.StartedAt}} RESTART={{.HostConfig.RestartPolicy.Name}}"'

ssh mower@<robot-ip> \
  'docker tag <old-image-id> mowglinext-gui:rollback-pre-<delta-tag>'
```

Never reuse or overwrite the old tag as the new image tag.

## 3. Extract and checksum the tested backend binary

```bash
mkdir -p /private/tmp/mowglinext-gui-delta
docker create --name mowgli-gui-extract mowglinext-gui:<delta-tag>
docker cp mowgli-gui-extract:/app/mowglinext \
  /private/tmp/mowglinext-gui-delta/mowglinext
docker rm mowgli-gui-extract
shasum -a 256 /private/tmp/mowglinext-gui-delta/mowglinext
```

Create a two-line delta Dockerfile:

```dockerfile
FROM mowglinext-gui:rollback-pre-<delta-tag>
COPY mowglinext /app/mowglinext
```

Create a compose override:

```yaml
services:
  gui:
    image: mowglinext-gui:<delta-tag>
```

## 4. Transfer and build the delta on the robot

```bash
ssh mower@<robot-ip> 'mkdir -p /home/mower/hil-evidence/<delta-tag>'
scp -p mowglinext Dockerfile gui-delta.override.yaml \
  mower@<robot-ip>:/home/mower/hil-evidence/<delta-tag>/

ssh mower@<robot-ip> '
  cd /home/mower/hil-evidence/<delta-tag>
  echo "<sha256>  mowglinext" | sha256sum -c -
  docker build -t mowglinext-gui:<delta-tag> .
'
```

The resulting image shares all large layers with the rollback image. Only the
backend binary is new.

## 5. Recreate only the GUI service

Read the active compose chain before changing anything:

```bash
ssh mower@<robot-ip> \
  'docker inspect mowgli-gui --format "{{index .Config.Labels \"com.docker.compose.project.working_dir\"}}|{{index .Config.Labels \"com.docker.compose.project.config_files\"}}"'
```

Run the same ordered `docker compose -f ...` chain and append the new override
last:

```bash
docker compose \
  -f <existing-file-1> \
  -f <existing-file-2> \
  -f /home/mower/hil-evidence/<delta-tag>/gui-delta.override.yaml \
  up -d --no-deps --force-recreate gui
```

Verify that the new container uses the expected image and retains
`restart: unless-stopped`.

## 6. Verify and roll back

Minimum verification:

- GUI HTTP endpoint returns 200.
- GUI connects to Foxglove and ROS data is visible.
- Browser reload succeeds repeatedly.
- Stateful plan/map/progress layers return after reload.
- Short network interruption recovers.
- GUI and Foxglove logs contain no new reconnect loop.
- CPU and memory are comparable to the pre-deploy snapshot.

Rollback by rerunning the same compose chain with a final override that selects:

```yaml
services:
  gui:
    image: mowglinext-gui:rollback-pre-<delta-tag>
```

Then recreate only `gui` with `--no-deps --force-recreate gui`.

## Frontend-only delta

Use this when all runtime changes are below `gui/web/**`. It builds and tests
the web app in Node, exports only `dist`, and creates one thin image layer on
top of the exact GUI image already running on the robot. It does not rebuild
the Go backend, firmware tooling, or ROS images.

Create a temporary build Dockerfile outside the repository:

```dockerfile
FROM node:22 AS build-web
COPY . /web
WORKDIR /web
RUN yarn install --frozen-lockfile
RUN yarn test src/hooks/multiplexedSocket.test.ts
RUN yarn build \
 && find dist/assets -type f \
      \( -name '*.js' -o -name '*.css' -o -name '*.json' \
         -o -name '*.svg' -o -name '*.wasm' \) \
      -size +1024c -exec gzip -9 -k {} +

FROM scratch AS export
COPY --from=build-web /web/dist /
```

Build and export only the static artifact. Use a fresh output directory because
BuildKit requires the destination not to contain an earlier export:

```bash
out="$(mktemp -d /private/tmp/mowgli-web-delta.XXXXXX)"
mkdir "$out/web"
docker build --target export \
  --output "type=local,dest=$out/web" \
  -f /private/tmp/MowgliNext.web-delta.Dockerfile \
  gui/web
```

Before changing the robot, record the current image ID and create a rollback
tag from that immutable ID as described above. Put the exported `web` directory
in a small archive, checksum it, and transfer it to a new evidence directory on
the robot.

Build the robot-side delta with:

```dockerfile
FROM mowglinext-gui:rollback-pre-<delta-tag>
COPY web/ /app/web/
```

Create a new tag rather than overwriting the base image. Append a compose
override selecting that tag to the robot's existing compose chain, then run:

```bash
docker compose -f <existing-files-in-order> \
  -f /home/mower/hil-evidence/<delta-tag>/gui-web-delta.override.yaml \
  up -d --no-deps gui
```

Verify the served `index.html` checksum, HTTP 200, Foxglove connection, repeated
map reloads, and the frontend-specific reconnect behavior. Rollback is the same
as for a backend delta: select the preserved rollback tag and recreate only
`gui`.
