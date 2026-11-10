# PC simulator: reference image -> palette -> strokes -> G-code + preview images.
#   make sim              -> build/sim/leogoya_sim
#   make run              -> paint the synthetic scene into build/sim/out/
#   make run REF=x.ppm    -> paint your own reference (convert with scripts/to_ppm.py)
# STM32 firmware lives in firmware/ (its own Makefile). scripts/build.sh drives both.

CC      ?= cc
CFLAGS  ?= -std=c11 -O2 -Wall -Wextra -Wno-unused-parameter
SRC      = $(wildcard sim/*.c sim/app/*.c sim/vision/*.c sim/planning/*.c sim/motion/*.c) \
           sim/hal/host/hal_host.c
BIN      = build/sim/leogoya_sim
OUT      = build/sim/out
REF     ?=

sim: $(BIN)

$(BIN): $(SRC) $(wildcard sim/*.h sim/*/*.h)
	@mkdir -p $(dir $(BIN))
	$(CC) $(CFLAGS) -Isim $(SRC) -lm -o $@

run: $(BIN)
	./$(BIN) $(REF) $(OUT)
	@command -v python3 >/dev/null && python3 scripts/preview.py $(OUT) || true

clean:
	rm -rf build/sim

.PHONY: sim run clean
