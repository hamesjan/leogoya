/*
 * config.h - every machine / canvas / painting constant in one place.
 *
 * Coordinate frames
 *   machine mm : what the STM32 controller (G-code) understands. Origin = homed corner.
 *   canvas mm  : origin at the canvas corner nearest machine origin, +x/+y same as machine.
 *   canvas px  : the planning image, WORK_RES x WORK_RES, (0,0) = canvas mm (0,0).
 *
 * Measure your build and edit the "MEASURE ME" values before the first run.
 */
#ifndef LEOGOYA_CONFIG_H
#define LEOGOYA_CONFIG_H

/* ---------------- Canvas ---------------- */
#define CANVAS_MM    965.2f /* 38 in square */
#define CANVAS_X0_MM 40.0f  /* MEASURE ME: canvas corner in machine coords */
#define CANVAS_Y0_MM 40.0f  /* MEASURE ME */

/* ---------------- Planning image ---------------- */
/* 256 px over 965 mm = 3.77 mm/px. Smallest brush is ~4 mm wide so finer
 * resolution buys nothing. 320+ is fine on a PC if you want finer detail. */
#define WORK_RES  256
#define MM_PER_PX (CANVAS_MM / (float)WORK_RES)

/* ---------------- Palette ---------------- */
#define PALETTE_N      6 /* paint pots on the palette station */
#define KMEANS_ITERS   24
#define KMEANS_SAMPLES 8192 /* random pixels used to fit k-means */

/* ---------------- Layers (coarse -> fine, Hertzmann 1998) ---------------- */
#define NUM_LAYERS 3
/* Brush width target per layer, mm. Each layer may use a different brush. */
#define LAYER_WIDTHS_MM                                                                                      \
    {                                                                                                        \
        24.0f, 12.0f, 5.0f                                                                                   \
    }
#define LAYER_BRUSH_ID                                                                                       \
    {                                                                                                        \
        0, 1, 2                                                                                              \
    } /* physical brush to load (manual swap) */
#define LAYER_ERR_THRESH                                                                                     \
    {                                                                                                        \
        18.0f, 12.0f, 8.0f                                                                                   \
    }                    /* mean deltaE in a grid cell to start a stroke */
#define BLUR_FACTOR 0.5f /* reference blur sigma = factor * brush radius */

/* ---------------- Strokes ---------------- */
#define STROKE_MAX_PTS        16
#define STROKE_MIN_PTS        3
#define STROKE_CURVATURE      0.6f /* 0 = rigid direction, 1 = follow gradient fully */
#define MAX_STROKES_PER_LAYER 4000
#define STROKE_OPACITY        0.85f /* sim alpha of fresh acrylic */
#define BRUSH_SOFT_EDGE_PX    0.6f

/* ---------------- Brush / Z (pressure) ---------------- */
/* Z is "press depth below touch-down", positive = more pressure.
 * Brush holder must be spring compliant (see docs/MECHANICAL.md). */
#define Z_SAFE_MM      25.0f /* travel height, machine Z */
#define Z_TOUCH_MM     10.0f /* MEASURE ME per brush: Z where bristles just touch */
#define Z_PRESS_MAX_MM 6.0f  /* never press deeper than this */
#define BRUSH_USES_YAW 1     /* 1 = flat/filbert brush, rotate to stroke tangent */
#define YAW_AXIS       'A'   /* G-code letter of the yaw axis on the STM32 controller */
#define LANDING_MM     4.0f  /* touch-down ramp length */
#define LIFTOFF_MM     6.0f  /* taper / flick length */
#define APPROACH_MM    3.0f  /* start this far behind stroke start */
#define PATH_STEP_MM   2.0f  /* spline resample spacing sent as G1 segments */

/* ---------------- Feeds (mm/min) ---------------- */
#define FEED_TRAVEL  9000
#define FEED_PAINT   3000
#define FEED_Z       1200
#define FEED_YAW_DEG 7200 /* degrees/min on the yaw axis */

/* ---------------- Paint load model ---------------- */
/* Area of canvas (mm^2) one dip covers before the stroke starts to go dry.
 * Calibrated by the brush routine; this is the first-guess default. */
#define PAINT_CAPACITY_MM2 9000.0f
#define RELOAD_FRACTION    0.75f /* reload when this fraction is used */

/* ---------------- Palette station (machine mm) - MEASURE ME ---------------- */
/* Pots in a row along +Y, to the right of the canvas (x > canvas). */
#define POT_X_MM       1060.0f
#define POT_Y0_MM      80.0f
#define POT_PITCH_MM   70.0f
#define POT_DIP_Z_MM   4.0f  /* machine Z for dipping into paint */
#define POT_RADIUS_MM  22.0f /* used for the "wipe on rim" move */
#define WATER_X_MM     1060.0f
#define WATER_Y_MM     (POT_Y0_MM + PALETTE_N * POT_PITCH_MM + 20.0f)
#define WATER_DIP_Z_MM 0.0f
#define SPONGE_X_MM    1060.0f
#define SPONGE_Y_MM    (WATER_Y_MM + 90.0f)
#define SPONGE_Z_MM    6.0f
#define PARK_X_MM      1060.0f /* out of camera view for captures */
#define PARK_Y_MM      1000.0f

/* ---------------- Camera / vision ---------------- */
#define MARKER_MIN_PIXELS 12 /* magenta corner dots */
#define CLOSED_LOOP       1  /* re-photograph the canvas after each layer */

#endif
