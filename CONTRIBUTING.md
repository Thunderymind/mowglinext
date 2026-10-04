# Contributing to MowgliNext

Thanks for your interest in contributing! MowgliNext is a community-driven project and we welcome contributions of all kinds.

## Getting Started

1. Fork the repository
2. Clone your fork **with submodules**: `git clone --recurse-submodules https://github.com/YOUR_USERNAME/mowglinext.git`
3. Create a feature branch off `dev`: `git checkout dev && git checkout -b feat/my-feature`
4. Make your changes
5. Push and open a Pull Request **against `dev`** (`gh pr create --base dev`) — `dev` is where feature work lands, `main` is the release branch

Branch prefixes matter: use `feat/`, `fix/`, `refactor/`, `chore/` or `perf/`. The CI push triggers match exactly those names, so a branch called anything else silently skips some checks.

### Fast feature loop

Keep the checkout and its worktrees in a **local, non-synced directory** (not iCloud Drive, OneDrive, or another cloud-sync folder). A Git worktree shares the repository's object store, so it gives each branch an isolated working directory without making another full clone. Before doing expensive work, confirm which checkout and branch you are in:

```bash
pwd -P
git rev-parse --show-toplevel
git status --short --branch
git worktree list
git remote -v
```

Then use this sequence to avoid spending time on the wrong baseline or repeating broad test runs:

1. **Confirm the base, image and remotes once.** Fetch the intended base (`dev` for feature work), make or rebase the feature worktree from it, and check `git remote -v` to identify the personal fork and upstream by URL—not by assuming a remote name. For ROS work, verify the Docker image tag and ROS distro/API before building; stale images can produce misleading compile failures. Don't rebuild or edit image definitions unless the selected image is actually missing the required baseline.
2. **Run the smallest useful test first.** Build the affected package/target and run the focused test, making sure `colcon test` is scoped to the intended source tree (include `--base-paths`). Once that passes, run the full affected-package suite, then the broader language/repository checks the change touches. Reuse the existing Docker build cache. In constrained environments, set `MAKEFLAGS=-j2` (or another measured cap): Colcon's `--parallel-workers` limits packages, not the Make jobs inside a package.
3. **Separate new failures from existing lint debt.** Run format/copyright/lint checks on changed files and compare broad lint findings with the base branch. Fix findings introduced by the diff; list unrelated pre-existing failures in the PR instead of expanding a small feature into a repository-wide cleanup.
4. **Commit, push and open the PR against `dev`.** Push only after confirming the exact fork URL and branch. Summarize the user-visible impact first and include measured before/after numbers (for example, payload size in MB and KB and the percentage reduction), plus an exact test ledger: what passed, what failed, and why. Don't describe a test as passing if it was not run or if its enclosing suite still has a failure.

For this repository, `dev` is the feature base and `main` is the release branch. A quick local change should not require recloning, rebuilding Docker images unnecessarily, or fixing unrelated lint debt.

## What to Contribute

- **Bug fixes** — found something broken? Fix it and send a PR
- **New sensor drivers** — add support for your GPS or LiDAR in `sensors/`
- **Behavior tree improvements** — new BT nodes or tree logic in `ros2/src/mowgli_behavior/`
- **GUI features** — React frontend or Go backend improvements in `gui/`
- **Documentation** — the user-facing pages live in `wiki/` (auto-synced to the GitHub wiki on merge to `main`); `docs/` holds the mowgli.garden landing page + install composer
- **Test coverage** — we need more tests across all packages
- **E2E test improvements** — `ros2/src/e2e_test.py`, `ros2/src/e2e_test_no_lidar.py`

## Finding Your Way Around

The repo carries a generated index of itself — the quickest way to locate something before you change it:

- [`CLAUDE.md`](CLAUDE.md) — safety rules, monorepo layout, architecture invariants. Each top-level component (`ros2/`, `gui/`, `firmware/`, `install/`, `docker/`, `sensors/`) has its own `CLAUDE.md` entry point.
- [`docs/claude/codemaps/`](docs/claude/codemaps/) — one map per package: files, runtime surface, known pitfalls
- [`docs/claude/ros-interfaces.md`](docs/claude/ros-interfaces.md) — every topic, service, action and TF frame, and the node that creates it
- [`docs/claude/parameters.md`](docs/claude/parameters.md) — every config key, its default and its consumers
- [`docs/claude/testing-ci.md`](docs/claude/testing-ci.md) — every test, the CI job that gates it, and a "before opening a PR" checklist
- [`docs/claude/doc-index.md`](docs/claude/doc-index.md) — which document is authoritative and which is history

## Development Setup

### ROS2 Stack

Requires ROS2 Lyrical on Ubuntu 26.04, plus two dependencies that have to be built from source:
GTSAM 4.3a1 (for `fusion_graph`) and Fields2Cover 3.0.0 (for `mowgli_coverage`). The recipes live in
`ros2/Dockerfile` (stages 0 and 0b) and are mirrored in `.github/workflows/ros2-ci.yml`. The
devcontainer and Codespaces image ship GTSAM, Fields2Cover v3 and the pinned Lyrical
source dependencies. The post-create hook leaves compilation opt-in. See
[the migration notes](docs/ROS2_LYRICAL_MIGRATION.md) for dependencies and validation.

```bash
git submodule update --init --recursive
cd ros2
source /opt/ros/lyrical/setup.bash
source /opt/lyrical_vendor/local_setup.bash
# Only opennav_coverage_msgs is used from the upstream submodule — the server
# subpackages are not built here (CI does the same).
for p in opennav_coverage opennav_coverage_bt opennav_coverage_demo \
         opennav_coverage_navigator opennav_row_coverage; do
  touch "src/opennav_coverage/$p/COLCON_IGNORE"
done
rosdep install --from-paths src --ignore-src --rosdistro lyrical -y \
  --skip-keys "grid_map_core grid_map_ros grid_map_msgs beluga_ros nav2_smac_planner webots_ros2_driver opennav_coverage opennav_coverage_bt opennav_coverage_demo opennav_coverage_navigator opennav_row_coverage"
colcon build
colcon test
```

### GUI

```bash
cd gui
go build -o openmower-gui
cd web && yarn && yarn build
```

### Firmware

```bash
cd firmware/stm32/ros_usbnode
pio run -e Yardforce500      # plain `pio run` builds this default env only
pio run -e Yardforce500B     # CI builds both boards
```

## Code Style

- **C++**: Follow `.clang-format` in `ros2/`. CI pins **clang-format 18** (`pip install clang-format==18.1.8`) and checks only the lines you changed — run `make -C ros2 format` before committing
- **Go**: Run `gofmt` (nothing in CI enforces it)
- **TypeScript/React**: Run `eslint` — `cd gui/web && yarn lint` (no Prettier config in the repo)
- **Python**: Follow PEP 8 (`ruff`, via `ros2/.pre-commit-config.yaml`)
- **Commits**: Use [conventional commits](https://www.conventionalcommits.org/) — `feat:`, `fix:`, `refactor:`, `docs:`, `test:`, `chore:`, `ci:`

## Pull Request Guidelines

- Open the PR against `dev`, not `main`
- Keep PRs focused — one feature or fix per PR
- Include a description of **what** and **why**
- Add tests for new functionality when possible
- Update documentation if your change affects user-facing behavior
- Flag anything that changes physical robot behavior as safety-critical in the description
- PRs are gated by CI and reviewed by a maintainer. There is no AI bot on the GitHub side — the Claude responder and automatic PR review workflows were removed in July 2026; AI assistance runs on your own machine (see [AI-Assisted Contributing](https://github.com/mowglinext/mowglinext/wiki/AI-Assisted-Contributing))
- The required status check on `dev` is `Build & Test (ROS2 kilted)`. The full "what will fail, in what order" list — including the code generators you owe after editing a `.msg`/`.srv` and the COBS protocol-version bump — is in [`docs/claude/testing-ci.md`](docs/claude/testing-ci.md#before-opening-a-pr)

## Reporting Bugs

Use the [bug report template](https://github.com/mowglinext/mowglinext/issues/new?template=bug_report.yml) and include:
- Hardware (board, GPS, LiDAR model)
- Steps to reproduce
- Expected vs actual behavior
- Relevant logs

## Proposing Features

Use the [feature request template](https://github.com/mowglinext/mowglinext/issues/new?template=feature_request.yml). The community and maintainers will help evaluate and refine proposals.

## License

By contributing, you agree that your contributions will be licensed under the project's [dual license](LICENSE): GPLv3 for open source, personal, educational, non-profit and community use, with a separate commercial license required for commercial use.
