# PC-side planner (photo -> palette -> strokes -> G-code). Runs on your laptop.
#   make planner          -> build/planner/leogoya_planner
#   make run              -> paint the synthetic scene into sim_out/
#   make run REF=x.ppm    -> paint your own reference (convert with scripts/to_ppm.py)
# STM32 firmware lives in firmware/ (its own Makefile). scripts/build.sh drives both.

CC      ?= cc
CFLAGS  ?= -std=c11 -O2 -Wall -Wextra -Wno-unused-parameter
SRC      = $(wildcard planner/*.c planner/app/*.c planner/vision/*.c planner/planning/*.c planner/motion/*.c) \
           planner/hal/host/hal_host.c
BIN      = build/planner/leogoya_planner
REF     ?=

planner: $(BIN)

$(BIN): $(SRC) $(wildcard planner/*.h planner/*/*.h)
	@mkdir -p $(dir $(BIN))
	$(CC) $(CFLAGS) -Iplanner $(SRC) -lm -o $@

run: $(BIN)
	./$(BIN) $(REF) sim_out
	@command -v python3 >/dev/null && python3 scripts/preview.py sim_out || true

clean:
	rm -rf build/planner sim_out

.PHONY: planner run clean
