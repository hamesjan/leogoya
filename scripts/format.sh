#!/usr/bin/env bash
# Apply the C style rules to firmware/ and planner/:
#   1. one variable per declaration   (clang-tidy readability-isolate-declaration, see .clang-tidy)
#   2. Allman braces on every if/else/for/while/do, indentation (clang-format, see .clang-format)
#
#   scripts/format.sh          fix everything in place
#   scripts/format.sh --check  report violations, change nothing (exit 1 if any)
set -euo pipefail
cd "$(dirname "$0")/.."

find_tool() {
    for candidate in "$1" "/opt/homebrew/opt/llvm/bin/$1" /opt/homebrew/opt/llvm@*/bin/"$1" \
                     "/Library/Developer/CommandLineTools/usr/bin/$1"; do
        if command -v "$candidate" >/dev/null 2>&1; then echo "$candidate"; return; fi
    done
    echo "error: $1 not found (brew install llvm)" >&2
    exit 1
}
CLANG_FORMAT=$(find_tool clang-format)
CLANG_TIDY=$(find_tool clang-tidy)

FIRMWARE_FLAGS=(-std=c11 --target=arm-none-eabi -mcpu=cortex-m4 -ffreestanding -Ifirmware/src)
PLANNER_FLAGS=(-std=c11 -Iplanner)

if [[ "${1:-}" == "--check" ]]; then
    status=0
    find firmware planner -name '*.[ch]' -print0 | xargs -0 "$CLANG_FORMAT" --dry-run --Werror || status=1
    for f in firmware/src/*.c; do "$CLANG_TIDY" --quiet --warnings-as-errors='*' "$f" -- "${FIRMWARE_FLAGS[@]}" || status=1; done
    for f in $(find planner -name '*.c'); do "$CLANG_TIDY" --quiet --warnings-as-errors='*' "$f" -- "${PLANNER_FLAGS[@]}" || status=1; done
    exit $status
fi

for f in firmware/src/*.c; do "$CLANG_TIDY" --quiet --fix "$f" -- "${FIRMWARE_FLAGS[@]}"; done
for f in $(find planner -name '*.c'); do "$CLANG_TIDY" --quiet --fix "$f" -- "${PLANNER_FLAGS[@]}"; done
find firmware planner -name '*.[ch]' -print0 | xargs -0 "$CLANG_FORMAT" -i
echo "formatted"
