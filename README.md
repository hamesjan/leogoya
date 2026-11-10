# Leogoya: a camera-guided acrylic painting robot

A 4-DOF gantry painter for a **38" × 38" canvas**, mostly 3D-printed. A ~$14 ESP32-S3 camera board photographs a reference, picks a paint palette with k-means, plans brush strokes, streams G-code to a cheap 3D-printer motion board, and re-photographs the canvas between layers to correct itself.

```
 reference ─► camera ─► rectify ─► k-means palette ─► stroke planner ◄─┐
                                                      │                │ canvas photo
                                                      ▼                │ (closed loop)
                                     spline + yaw + pressure profile   │
                                                      ▼                │
                         UART G-code ─► SKR Mini E3 (Marlin) ─► X Y Z yaw ─► paint
```

| | |
|---|---|
| **DOF** | X, Y (gantry) + Z press depth (spring-compliant) + brush yaw. Why 4: [docs/MECHANICAL.md](docs/MECHANICAL.md) |
| **Brain MCU** | Seeed XIAO ESP32S3 Sense: 8 MB PSRAM, 8 MB flash, OV2640 |
| **Motion** | BTT SKR Mini E3 V3 (4× TMC2209) running Marlin 2.1 |
| **Frame** | 3/4" EMT conduit + printed roller carriages (608 bearings) |
| **Cost** | ≈ $410 required parts incl. tax/shipping, cheapest sources ([BOM](hardware/leogoya_BOM.xlsx)) |

## Repo layout

```
src/                       portable firmware (C11), runs on the ESP32-S3 and on your laptop
  main.c                   entry point (app_main on ESP-IDF, main() on host)
  config.h                 ALL machine / canvas / painting constants (measure + edit these)
  app/                     application logic
    app.c/.h               state machine: home → reference → palette → layers (closed loop) → done
    bootloader.c/.h        boot policy: crash-loop safe mode, OTA rollback confirm, calibrate-at-boot
  vision/                  camera → understanding the image
    vision.c/.h            magenta corner markers → homography → top-down Lab image
    image.c/.h             Lab color, blur, gradient
    palette.c/.h           k-means++ palette (the "which paints" ML step) + mixing hints
  planning/                deciding what to paint
    planner.c/.h           Hertzmann layered strokes + greedy simulator acceptance
    brush_model.c/.h       width-vs-pressure regression (learned on device), paint-load model
    stroke.h               stroke data shared by planning and motion
  motion/                  turning strokes into machine moves
    motion.c/.h            stroke → Catmull-Rom → G-code with yaw, landing/lift-off, dip/rinse
  hal/                     hardware abstraction layer
    hal.h                  interface: camera, UART, button, storage, PSRAM
    esp32/hal_esp32.c      XIAO ESP32S3 Sense implementation
    host/hal_host.c        simulator: fake camera, G-code to file, PPM previews
  common/util.h            PRNG, clamp
main/                    ESP-IDF component glue (pulls in src/)
hardware/leogoya_BOM.xlsx  BOM, cheapest vs alt sources, printed parts, design options
docs/                    MECHANICAL, WIRING (+ Marlin config), ALGORITHMS (ideas + roadmap)
tools/                   preview.py (PPM → PNG montage), to_ppm.py (any image → reference)
```

## Quick start: simulator (no hardware)

```sh
make run                                   # paints a synthetic scene into sim_out/
python3 tools/to_ppm.py photo.jpg ref.ppm  # needs Pillow
make run REF=ref.ppm                       # paint your own reference
open sim_out/montage.png                   # reference | after layer 0 | 1 | 2
```
`sim_out/painting.gcode` is the exact G-code the robot would run. It's handy for a dry run in a G-code viewer.

## Quick start: hardware

```sh
. $IDF_PATH/export.sh          # ESP-IDF v5.3+
idf.py set-target esp32s3
idf.py build flash monitor
```
Then follow the bring-up checklist in [docs/WIRING.md](docs/WIRING.md). Operator flow (one button):
1. Power on. It homes. Hold the button at boot to run **brush calibration** instead.
2. Put the reference print (with 4 magenta corner dots) under the camera and press.
3. The serial log prints the palette (hex + mixing hint). Mix and fill the pots, load brush 0, press.
4. It paints layer by layer and asks you to swap brushes between layers. If power is lost it resumes at the last unfinished layer.

## Memory budget (ESP32-S3, 8 MB PSRAM)

| Buffer | Size |
|---|---|
| Camera frame 800×600 RGB565 | 938 KB |
| Reference / blurred ref / sim canvas / canvas photo (4 × 256² Lab float) | 3 MB |
| Blur scratch + stroke coverage mask | 512 KB |
| Stroke seeds + 4000 strokes | ~0.9 MB |
| k-means scratch (transient) | 136 KB |
| **Total** | **≈ 5.5 MB of 8 MB** |

"One picture" is the reference. It's also saved to flash (`storage` partition) so a reboot doesn't need it again.

## Status

- Host simulator builds with no warnings and runs end to end (test scene: mean ΔE 54.7 → 4.3 over 3 layers, ~2.3k strokes).
- The ESP32 port (`hal_esp32.c`, CMake, sdkconfig, partitions) is written against the ESP-IDF 5.3 + esp32-camera 2.x APIs but **has not been compiled or run on hardware yet**. Expect small fixes on the first `idf.py build`.
- Marlin I-axis pin mapping: verify for your Marlin version (see WIRING.md).
- BOM prices are estimates. Verify before ordering.

Next steps and algorithm ideas: [docs/ALGORITHMS.md](docs/ALGORITHMS.md).
