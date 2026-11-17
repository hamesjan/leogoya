"""Plan a brush square on the canvas, check it with forward kinematics, and plot it.

    uv run python -m leogoya.plan_square
    uv run python -m leogoya.plan_square --size 0.12 --center 0.22 0.0 --brush-length 0.06

Writes to build/out/:
    square_joints.csv  one row per control tick, columns = SO101Follower action keys (degrees)
    square.png         top view (reachable area + path), side view, joint angles over time
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .arm import ARM_JOINTS, HOME, BrushArm

POS_TOL_MM = 1.0
TILT_TOL_DEG = 2.0
MAX_STEP_DEG = 5.0  # biggest joint change allowed between ticks; larger means an IK flip


def line(a: np.ndarray, b: np.ndarray, step: float) -> np.ndarray:
    n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
    return a + (b - a) * np.linspace(0, 1, n + 1)[1:, None]


def square_path(center, size, canvas_z, lift, step) -> tuple[np.ndarray, np.ndarray]:
    """Brush-tip points (N, 3) and a pen-down flag per point."""
    cx, cy = center
    h = size / 2
    corners = [np.array([cx + dx, cy + dy, canvas_z]) for dx, dy in [(-h, -h), (h, -h), (h, h), (-h, h), (-h, -h)]]
    up = np.array([0, 0, lift])
    above_start = corners[0] + up
    pts: list[np.ndarray] = [above_start[None]]
    down: list[bool] = [False]

    def add(seg: np.ndarray, flags: list[bool]) -> None:
        pts.append(seg)
        down.extend(flags)

    lower = line(above_start, corners[0], step)
    add(lower, [False] * (len(lower) - 1) + [True])  # touches down on the last point
    for a, b in zip(corners, corners[1:], strict=False):
        side = line(a, b, step)
        add(side, [True] * len(side))
    lift_off = line(corners[-1], corners[-1] + up, step)
    add(lift_off, [False] * len(lift_off))
    return np.vstack(pts), np.array(down)


def reachable_grid(arm: BrushArm, canvas_z: float, res: float = 0.02):
    xs = np.arange(0.04, 0.42, res)
    ys = np.arange(-0.32, 0.32 + 1e-9, res)
    ok = np.zeros((len(ys), len(xs)), bool)
    def good(s) -> bool:
        return s.pos_error_mm < POS_TOL_MM and s.tilt_deg < TILT_TOL_DEG

    # Cold-start IK sometimes stalls, so try the neighbour's solution and HOME; take the better one.
    for i, y in enumerate(ys):
        prev = None
        for j, x in enumerate(xs):
            tip = np.array([x, y, canvas_z])
            tries = [arm.solve(tip, arm_seed(x, y), iters=80)]
            if prev is not None:
                tries.append(arm.solve(tip, prev, iters=80))
            best = min(tries, key=lambda s: (not good(s), s.pos_error_mm))
            ok[i, j] = good(best)
            prev = best.joints if ok[i, j] else None
    return xs, ys, ok


def arm_seed(x: float, y: float) -> np.ndarray:
    """HOME pose turned toward (x, y)."""
    seed = HOME.copy()
    seed[0] = np.degrees(np.arctan2(y, x))
    return seed


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--center", type=float, nargs=2, default=[0.20, 0.0], metavar=("X", "Y"), help="m")
    ap.add_argument("--size", type=float, default=0.10, help="square side, m")
    ap.add_argument("--canvas-z", type=float, default=0.0, help="canvas surface height above the base, m")
    ap.add_argument("--lift", type=float, default=0.02, help="pen-up height above the canvas, m")
    ap.add_argument("--brush-length", type=float, default=0.0, help="bristle tip past the jaw tips, m")
    ap.add_argument("--speed", type=float, default=0.05, help="brush speed, m/s")
    ap.add_argument("--fps", type=int, default=30, help="control rate the CSV is sampled at")
    ap.add_argument("--out", type=Path, default=Path("build/out"))
    args = ap.parse_args()

    arm = BrushArm(brush_length=args.brush_length)
    tips, down = square_path(args.center, args.size, args.canvas_z, args.lift, step=args.speed / args.fps)
    sols = arm.solve_path(tips, seed=arm_seed(*tips[0][:2]))
    q = np.array([s.joints for s in sols])
    err = np.array([s.pos_error_mm for s in sols])
    tilt = np.array([s.tilt_deg for s in sols])
    jump = np.abs(np.diff(q, axis=0)).max() if len(q) > 1 else 0.0
    lo, hi = arm.limits_deg[:, 0], arm.limits_deg[:, 1]
    in_limits = bool(np.all((q >= lo - 1e-6) & (q <= hi + 1e-6)))

    print(f"{len(tips)} ticks @ {args.fps} Hz = {len(tips) / args.fps:.1f} s")
    print(f"position error  max {err.max():.3f} mm   (tol {POS_TOL_MM})")
    print(f"brush tilt      max {tilt.max():.2f} deg  (tol {TILT_TOL_DEG})")
    print(f"joint step      max {jump:.2f} deg/tick (tol {MAX_STEP_DEG})")
    print(f"joint limits    {'ok' if in_limits else 'VIOLATED'}")
    for name, col in zip(ARM_JOINTS, q.T, strict=True):
        print(f"  {name:13s} {col.min():7.1f} .. {col.max():7.1f} deg")
    good = err.max() < POS_TOL_MM and tilt.max() < TILT_TOL_DEG and jump < MAX_STEP_DEG and in_limits
    print("PLAN OK" if good else "PLAN FAILED: move/shrink the square or change --canvas-z / --brush-length")

    args.out.mkdir(parents=True, exist_ok=True)
    csv_path = args.out / "square_joints.csv"
    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", *[f"{j}.pos" for j in ARM_JOINTS], "brush_down"])
        for i, (row, d) in enumerate(zip(q, down, strict=True)):
            w.writerow([f"{i / args.fps:.4f}", *[f"{v:.3f}" for v in row], int(d)])

    print("computing reachable canvas area...")
    xs, ys, ok = reachable_grid(arm, args.canvas_z)
    reached = np.array([arm.brush_tip(j)[0] for j in q])

    fig, ax = plt.subplots(1, 3, figsize=(17, 5.2))
    a = ax[0]
    a.pcolormesh(xs, ys, ok, cmap="Greens", alpha=0.35, shading="nearest", vmin=0, vmax=1.4)
    a.plot(reached[down, 0], reached[down, 1], "-", color="tab:blue", lw=2, label="brush down")
    a.plot(tips[~down, 0], tips[~down, 1], ".", color="tab:orange", ms=3, label="brush up")
    a.plot(0, 0, "ks", ms=10, label="robot base")
    a.set(title=f"Top view — green = reachable with brush vertical at z={args.canvas_z} m",
          xlabel="x (m, forward)", ylabel="y (m, left)", aspect="equal")
    a.legend(loc="upper left", fontsize=8)
    a = ax[1]
    t = np.arange(len(tips)) / args.fps
    a.plot(t, reached[:, 2] * 1000, label="brush tip z (mm)")
    a.plot(t, err, label="position error (mm)")
    a.plot(t, tilt, label="tilt (deg)")
    a.set(title="Brush height, IK error and tilt", xlabel="time (s)")
    a.legend(fontsize=8)
    a = ax[2]
    for name, col in zip(ARM_JOINTS, q.T, strict=True):
        a.plot(t, col, label=name)
    a.set(title="Joint commands (degrees, = SO101Follower actions)", xlabel="time (s)", ylabel="deg")
    a.legend(fontsize=8)
    fig.tight_layout()
    png = args.out / "square.png"
    fig.savefig(png, dpi=110)
    print(f"wrote {csv_path} and {png}")


if __name__ == "__main__":
    main()
