# STM32 controller firmware (Nucleo-F401RE). Needs only arm-none-eabi-gcc:
#   brew install arm-none-eabi-gcc arm-none-eabi-binutils
#
#   make            -> ../build/firmware/leogoya.elf / .bin
#   make flash      -> copy .bin to the Nucleo's USB drive (or st-flash if installed)
#   make size

PREFIX  ?= arm-none-eabi-
CC      := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size

OUT     := ../build/firmware
TARGET  := $(OUT)/leogoya
SRC     := $(wildcard src/*.c)
OBJ     := $(patsubst src/%.c,$(OUT)/%.o,$(SRC))
LDSCRIPT:= stm32f401re.ld

ARCH    := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS  := $(ARCH) -std=c11 -Os -g3 -Wall -Wextra -Werror \
           -ffreestanding -fno-builtin -fno-tree-loop-distribute-patterns \
           -ffunction-sections -fdata-sections -MMD -MP
LDFLAGS := $(ARCH) -T$(LDSCRIPT) -nostdlib -Wl,--gc-sections -Wl,-Map=$(TARGET).map

all: $(TARGET).bin size

$(OUT)/%.o: src/%.c | $(OUT)
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJ) $(LDSCRIPT)
	$(CC) $(LDFLAGS) $(OBJ) -lgcc -o $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(OUT):
	mkdir -p $@

size: $(TARGET).elf
	@$(SIZE) $<

# Nucleo boards mount as a USB drive (NODE_F401RE, NODE_F411RE, ...): copying
# a .bin onto it flashes the chip. Falls back to st-flash if no drive is found.
flash: $(TARGET).bin
	@drive=$$(ls -d /Volumes/NODE_* 2>/dev/null | head -1); \
	if [ -n "$$drive" ]; then echo "copying to $$drive"; cp $< "$$drive/" && sync; \
	elif command -v st-flash >/dev/null; then st-flash --reset write $< 0x08000000; \
	else echo "no Nucleo drive mounted and st-flash not installed (brew install stlink)"; exit 1; fi

clean:
	rm -rf $(OUT)

-include $(OBJ:.o=.d)

.PHONY: all size flash clean
