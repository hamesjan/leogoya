# Wiring and motion-board setup

## Block diagram

```
 mains ─[fused IEC inlet]─ 24 V PSU ─┬─ SKR Mini E3 V3 (VIN 24 V)
                                     └─ LM2596 buck → 5 V ─ XIAO 5V pin
                                                                │
   XIAO ESP32S3 Sense (camera, on the mast)                     │
     D6 / GPIO43 (TX) ───────────────► TFT header RX            │
     D7 / GPIO44 (RX) ◄─────────────── TFT header TX            │
     GND ───────────────────────────── TFT header GND ──────────┘ (common ground!)
     D1 / GPIO2 ── button ── GND
     USB-C → laptop (flashing / log console)

   SKR Mini E3 V3
     X  driver → X motor
     Y  driver → Y1 + Y2 motors (parallel splitter; one motor's connector reversed so they turn opposite ways if mounted mirrored)
     Z  driver → Z lead-screw motor  (Z-stop = slide touch microswitch, optional)
     E  driver → yaw pancake motor
     X-stop / Y-stop → microswitches (or TMC2209 sensorless homing)
```
- Both boards use 3.3 V logic, so connect UART directly. **Cross TX/RX** and share GND.
- The TFT header also has a 5 V pin. It can power the XIAO instead of the buck, but the camera plus Wi-Fi peaks are better on their own regulator.
- The UART cable runs ~1.5 m up the mast. Keep it at 115200 baud and twist TX/RX with GND.

## Marlin 2.1.x configuration (SKR Mini E3 V3)

Start from `config/examples/BigTreeTech/SKR Mini E3 3.0` in the MarlinFirmware/Configurations repo, then:

### Option A (preferred): yaw as a real rotary axis
```c
// Configuration.h
#define SERIAL_PORT 2            // TFT header, talks to the XIAO
#define SERIAL_PORT_2 -1         // USB, for Pronterface jogging
#define BAUDRATE 115200
#define EXTRUDERS 0              // no hotend, no temp sensors
#define TEMP_SENSOR_0 0
#define TEMP_SENSOR_BED 0
#define I_DRIVER_TYPE TMC2209
#define AXIS4_NAME 'A'
#define AXIS4_ROTATES
#define DEFAULT_AXIS_STEPS_PER_UNIT { 80, 80, 400, 8.8889 }  // X Y (GT2 20T, 1/16), Z (T8x8), A (deg)
#define DEFAULT_MAX_FEEDRATE        { 300, 300, 20, 360 }
#define DEFAULT_MAX_ACCELERATION    { 1500, 1500, 200, 3000 }
#define X_BED_SIZE 1100
#define Y_BED_SIZE 1010
#define Z_MAX_POS 40
#define I_MIN_POS -720
#define I_MAX_POS 720
// Route the I (A) axis to the board's E0 driver socket. The I_* pins must
// map to the E0 pins (and the E0 TMC UART address); add these where your
// Marlin version expects pin overrides, e.g. the end of Configuration.h:
#define I_STEP_PIN   E0_STEP_PIN
#define I_DIR_PIN    E0_DIR_PIN
#define I_ENABLE_PIN E0_ENABLE_PIN
```
Verify against your Marlin version: pin-override placement and the TMC2209 UART address for `I` (the E0 socket is address 3) have moved between releases. Compile, then send `M122` and check that 4 drivers respond.

### Option B (fallback): yaw on the extruder axis
Keep `EXTRUDERS 1`, set E steps to 8.8889 steps/deg, and set `YAW_AXIS 'E'` in `src/config.h` (the firmware then sends `M302 P1` to allow cold extrusion). Marlin will still want a hotend thermistor, so put a 100 kΩ resistor across TH0 to read ~25 °C, and set `HEATER_0_MINTEMP` low. It's a hack, but it uses a stock build.

## Bring-up checklist
1. SKR only, USB to laptop, Pronterface: home X/Y, jog every axis, and check the direction and steps/mm with a ruler.
2. Measure and write into `src/config.h`: `CANVAS_X0/Y0_MM`, pot positions, `WATER_*`, `SPONGE_*`, `PARK_*`, `Z_TOUCH_MM` (jog the brush down until it just touches).
3. XIAO alone: `idf.py flash monitor`, and confirm the camera inits and PSRAM shows 8 MB.
4. Connect the UART. The first boot should home the machine (`G28`) and then ask for the reference.
5. Dry run with no paint and the brush 5 mm high (temporarily raise `Z_TOUCH_MM`).
