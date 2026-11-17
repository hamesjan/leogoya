# Leogoya

Painting robot on an SO-101 arm, using [LeRobot](https://github.com/huggingface/lerobot) for motors, calibration and kinematics.

## Setup

Expects the LeRobot clone at `../lerobot` (see `[tool.uv.sources]` in `pyproject.toml`).

```sh
scripts/dev.sh build    # or: uv sync
uv run lerobot-find-port
```

## Layout

- `hardware/` — SO-101 leader/follower STLs and STEP assembly

## Plan a square (no hardware needed)

```sh
uv run python -m leogoya.plan_square                      # 10 cm square, 20 cm in front of the base
uv run python -m leogoya.plan_square --brush-length 0.05  # bristle tip 5 cm past the jaw tips
open build/out/square.png
```

Plans brush-tip waypoints on the canvas, solves IK with the brush vertical, re-checks every
point with forward kinematics and writes `build/out/square_joints.csv`, whose columns are SO101Follower action keys
(degrees, sampled at `--fps`). Frame: origin at the robot base, +x forward, +y left, +z up, table at z = 0.

- `src/leogoya/arm.py` — `BrushArm`: brush-tip position → joint angles, and back
- `hardware/so101_urdf/` — SO-101 URDF + meshes from TheRobotStudio/SO-ARM100 (Apache-2.0, see `SOURCE`)
