# Leogoya

## Build

```sh
brew install arm-none-eabi-gcc arm-none-eabi-binutils   # once
scripts/build.sh            # firmware + simulator
scripts/build.sh flash      # flash the Nucleo (copies the .bin to its USB drive)
```

## Talk to the board

```sh
python3 -m pip install pyserial
python3 scripts/send.py                          # interactive
# or: screen /dev/cu.usbmodem* 115200, then send ECHO ON so you can see what you type
```

```
> PING
pong
ok
> G1 X100 Y50 F3000
ok
> M114
X:100.000 Y:50.000 Z:0.000 A:0.000 F:3000 ABS
ok
```

## Paint (simulator → board)

```sh
scripts/build.sh run                          # plan the synthetic test scene → build/sim/out/
python3 scripts/to_ppm.py photo.jpg ref.ppm   # your own reference (needs Pillow)
scripts/build.sh run ref.ppm
open build/sim/out/montage.png                # reference | after each layer
python3 scripts/send.py build/sim/out/painting.gcode
```
