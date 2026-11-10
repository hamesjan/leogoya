# Host simulator build (no hardware needed).
#   make sim            -> build/leogoya_sim
#   make run            -> paint the synthetic scene into sim_out/
#   make run REF=x.ppm  -> paint your own reference (convert with tools/to_ppm.py)
# Firmware for the ESP32-S3 is built with ESP-IDF instead: `idf.py build`.

CC      ?= cc
CFLAGS  ?= -std=c11 -O2 -Wall -Wextra -Wno-unused-parameter
# portable code + the host HAL (the ESP32 HAL is only built by ESP-IDF)
SRC      = $(wildcard src/*.c src/app/*.c src/vision/*.c src/planning/*.c src/motion/*.c) \
           src/hal/host/hal_host.c
BIN      = build/leogoya_sim
REF     ?=

sim: $(BIN)

$(BIN): $(SRC) $(wildcard src/*.h src/*/*.h)
	@mkdir -p build
	$(CC) $(CFLAGS) -Isrc $(SRC) -lm -o $@

run: $(BIN)
	./$(BIN) $(REF) sim_out
	@command -v python3 >/dev/null && python3 tools/preview.py sim_out || true

clean:
	rm -rf build sim_out

.PHONY: sim run clean
