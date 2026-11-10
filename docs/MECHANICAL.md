# Mechanical design

## Degrees of freedom: why exactly 4

A brush stroke on a flat canvas is fully described by:

| Stroke property | Needs | Axis |
|---|---|---|
| Where the stroke goes (path on the canvas) | 2 DOF | **X, Y** |
| How wide / heavy it is (pressure) | 1 DOF | **Z**, through a spring-compliant holder |
| Which way a flat/filbert brush faces | 1 DOF | **Yaw** about the brush axis |

**Not needed:** brush tilt (pitch/roll). Painters tilt to see the canvas and for wrist comfort; the bristles' bend under pressure already gives the angled contact. Two extra tilt axes would add weight, cost and calibration with little visible gain. If you only use round brushes, set `BRUSH_USES_YAW 0` and drop to **3 DOF**.

## Why a gantry and not an arm

A 38" (965 mm) canvas has a 1.37 m diagonal. A printed SCARA or articulated arm with that reach puts big bending moments on printed joints. The brush tip moves millimetres when it presses, and accuracy changes with pose. A gantry on steel tubes is stiffer, cheaper, has trivial kinematics, and gives the same accuracy at every point. The "arm" becomes the small 2-DOF wrist (Z + yaw) riding on the gantry.

## Layout (top view, ~1.2 × 1.2 m on a 4×4 ft base board)

```
  Y ▲
    │  ┌─────────────────────────────────────────────┬──────────┐
    │  │ Y rail (EMT) ═══════════════════════════════│══════════│
    │  │                                             │  sponge  │
    │  │                                             │  water   │
    │  │          38" x 38" canvas                    │  pot 5   │
    │  │          (horizontal, face up)              │  pot 4   │
    │  │                                             │  pot 3   │
    │  │        X gantry (2 EMT tubes) ═══[toolhead]═│══ pot 2  │
    │  │                                             │  pot 1   │
    │  │                                             │  pot 0   │
    │  │ Y rail (EMT) ═══════════════════════════════│══════════│
    │  └─────────────────────────────────────────────┴──────────┘
    └──────────────────────────────────────────────────────────► X
       ^ home (0,0)    canvas origin = (CANVAS_X0, CANVAS_Y0)     ^ palette station x ≈ 1060
```
- **Horizontal canvas.** Acrylic thinned enough to flow will run on a vertical easel.
- **Palette station** sits inside the gantry envelope (x > canvas), so dipping is just another move.
- Travel needed is about X 1100 mm and Y 1010 mm. Rails are cut ~1.35 m.

## Toolhead (the "wrist")

```
     X carriage plate (rolls on 2 X tubes, 6× 608)
          │
     [NEMA17 + T8 lead screw]  ← Z: sets press depth
          │ brass nut carrier
     ┌────┴────┐   two 4 mm guide pins
     │ spring  │   ← compliance: Z position → force
     │ slide   │   microswitch here = touch probe (sag map)
     └────┬────┘
     [pancake NEMA17]          ← yaw, shaft down
          │
     [brush collet] ── brush
```
- **Spring compliance is essential.** Without it, Z error (canvas sag, warp, lead-screw backlash) shows up directly as width error, and a stiff brush gouges. With a ~0.3 N/mm spring, 1 mm of Z error is a small force change. Pick the spring so a full `Z_PRESS_MAX_MM` press bends the brush noticeably without bottoming the slide.
- **Touch probe:** a microswitch that trips when the slide starts to compress gives you Marlin `G29` bilinear leveling over the canvas sag. See ALGORITHMS.md §3.6.
- **Collets:** print one collet per brush, bored to that handle. A brush swap is then a 10-second job with repeatable length, so `z_touch` per brush stays valid.
- Keep the moving mass under ~500 g so 40 mm NEMA17s with belts are plenty.

## Camera mount

- Mast: vertical EMT at a canvas corner with a printed arm, so the XIAO sits above the canvas center, lens down.
- Height to see the whole canvas plus the corner markers: `h ≈ (diagonal/2 + margin) / tan(FOV_diag/2)`.
  - Stock OV2640 lens (~66° diagonal): h ≈ 0.75 / tan(33°) ≈ **1.15 m**.
  - 120° wide lens: h ≈ 0.45 m (but expect barrel distortion, see below).
- The gantry parks at `PARK_X/Y` (over the palette station, out of frame) before every photo.
- **Lighting:** two LED bars at ~45° on opposite sides, fixed, no window light. AE/AWB is locked on the first canvas photo (`src/hal/esp32/hal_esp32.c`).
- Wide lenses: the 4-point homography doesn't correct barrel distortion. Either use the stock lens, or add a radial-distortion step (k1, k2 from a checkerboard) before `vision_rectify`.

## Markers

Bright magenta ~15–20 mm dots:
- **Reference:** stick them at the 4 corners of the print (just outside the image is best), and lay it under the camera.
- **Canvas:** stick them on the stretcher-bar corners, outside the painted area, so paint never covers them.
- Check them before trusting the camera: `vision_find_markers` logs pixel counts.

## Printed parts

See `hardware/leogoya_BOM.xlsx` → *Printed Parts* (~1.6 kg PETG, ~75 h). Use PETG (water, paint, belt tension creep) with 4 walls on anything bolted to a tube or belt. Use heat-set inserts, not threads in plastic, on the toolhead.

## Canvas size note

38×38 isn't a stock size. Stretch your own (cheapest), or use 36×36 and set `CANVAS_MM 914.4f`. Repeatable placement matters: print the corner locators and screw them to the base board.
