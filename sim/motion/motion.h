/*
 * motion.h - turns planned strokes into G-code for the STM32 controller.
 *
 * Axes (4 DOF, the minimum for a directional brush stroke):
 *   X, Y : brush position over the canvas
 *   Z    : press depth -> stroke width (spring-compliant brush holder)
 *   A/E  : brush yaw, keeps a flat/filbert brush aligned with the stroke tangent
 *
 * The STM32 controller (firmware/) executes the G-code; we only emit short
 * G1 segments and let it handle step generation and acceleration.
 */
#ifndef LEOGOYA_MOTION_H
#define LEOGOYA_MOTION_H

#include "planning/brush_model.h"
#include "planning/stroke.h"
#include <stdbool.h>

bool motion_init(void); /* configure + home */
bool motion_park(void); /* move out of the camera's view */
bool motion_load_paint(int pot, bool rinse_first);
bool motion_rinse(void);
bool motion_paint_stroke(const stroke_t *stroke, const brush_t *brush);

/* calibration helper: straight line at constant press depth, canvas mm */
bool motion_paint_line(float start_x, float start_y, float end_x, float end_y, float depth_mm,
                       const brush_t *brush);

/* total stroke-path length emitted so far, mm (stats) */
float motion_paint_distance(void);

#endif
