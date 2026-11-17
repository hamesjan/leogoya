#!/usr/bin/env bash
# Leogoya dev tasks.
#
#   scripts/dev.sh sync          install/update the env (uv sync, lerobot from ../lerobot)
#   scripts/dev.sh check         make sure lerobot, placo and the URDF load
#   scripts/dev.sh build         sync + check
#   scripts/dev.sh plan [args]   plan the test square -> build/out/ (args go to leogoya.plan_square)
#   scripts/dev.sh port          find the arm's USB port (interactive, needs the arm plugged in)
#   scripts/dev.sh clean         delete build/ and Python caches
set -euo pipefail
SELF="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"  # absolute, so it survives the cd below
cd "$(dirname "$SELF")/.."

log()  { printf '\033[1;34m==>\033[0m %s\n' "$*"; }
fail() { printf '\033[1;31merror:\033[0m %s\n' "$*" >&2; exit 1; }

sync() {
    command -v uv >/dev/null || fail "uv not found: brew install uv"
    [[ -d ../lerobot ]] || fail "expected the lerobot clone at ../lerobot (see pyproject.toml)"
    log "uv sync"
    uv sync
}

check() {
    log "checking imports and URDF"
    # placo prints a self-collision warning for the URDF's neutral pose; it's harmless.
    uv run python -c "
from leogoya.arm import BrushArm
import numpy as np
arm = BrushArm()
tip, tilt = arm.brush_tip(np.zeros(5))
print(f'ok: SO-101 model loaded, zero-pose tip at {np.round(tip, 3)} m')
" 2>&1 | grep -v -e '^WARNING: Robot has' -e 'collides with'
}

case "${1:-help}" in
    sync)  sync ;;
    check) check ;;
    build) sync; check ;;
    plan)  shift; uv run python -m leogoya.plan_square "$@" 2>&1 | grep -v -e '^WARNING: Robot has' -e 'collides with' ;;
    port)  uv run lerobot-find-port ;;
    clean) log "removing build/ and caches"; rm -rf build; find . -name __pycache__ -not -path './.venv/*' -prune -exec rm -rf {} + ;;
    -h|--help|help) sed -n '2,9p' "$SELF" | sed 's/^# \{0,1\}//' ;;
    *) fail "unknown command '$1' (try: scripts/dev.sh help)" ;;
esac
