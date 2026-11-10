#!/usr/bin/env bash
# Build Leogoya.
#
#   scripts/build.sh               STM32 firmware + PC simulator
#   scripts/build.sh firmware      STM32 firmware only   -> build/firmware/leogoya.bin
#   scripts/build.sh sim           PC simulator only     -> build/sim/leogoya_sim
#   scripts/build.sh run [x.ppm]   build the simulator and paint into build/sim/out/
#   scripts/build.sh flash         build + flash the Nucleo (USB drive copy or st-flash)
#   scripts/build.sh clean         delete build/
set -euo pipefail

cd "$(dirname "$0")/.."

log()  { printf '\033[1;34m==>\033[0m %s\n' "$*"; }
fail() { printf '\033[1;31merror:\033[0m %s\n' "$*" >&2; exit 1; }

build_firmware() {
    command -v arm-none-eabi-gcc >/dev/null 2>&1 ||
        fail "arm-none-eabi-gcc not found. Install it: brew install arm-none-eabi-gcc arm-none-eabi-binutils"
    log "building STM32 firmware"
    make --no-print-directory -C firmware
}

build_sim() {
    log "building PC simulator"
    make --no-print-directory sim
}

case "${1:-all}" in
    all)      build_firmware; build_sim ;;
    firmware) build_firmware ;;
    sim)      build_sim ;;
    run)
        build_sim
        log "painting into build/sim/out/"
        make --no-print-directory run REF="${2:-}"
        ;;
    flash)
        build_firmware
        log "flashing"
        make --no-print-directory -C firmware flash
        ;;
    clean)
        log "removing build/"
        rm -rf build
        ;;
    -h|--help|help) sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//' ;;
    *) fail "unknown command '$1' (try: scripts/build.sh help)" ;;
esac
