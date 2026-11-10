/*
 * motion.h - turns planned strokes into G-code for the motion board.
 *
 * Axes (4 DOF, the minimum for a directional brush stroke):
 *   X, Y : brush position over the canvas
 *   Z    : press depth -> stroke width (spring-compliant brush holder)
 *   A/E  : brush yaw, keeps a flat/filbert brush aligned with the stroke tangent
 *
 * The motion board (BTT SKR Mini E3 V3 running Marlin 2.1.x) does step
 * generation, acceleration and lookahead; we only emit short G1 segments.
 */
#ifndef LEOGOYA_MOTION_H
#define LEOGOYA_MOTION_H

#include "planning/brush_model.h"
#include "planning/stroke.h"
#include <stdbool.h>

bool motion_init(void);                 /* configure + home */
bool motion_park(void);                 /* move out of the camera's view */
bool motion_load_paint(int pot, bool rinse_first);
bool motion_rinse(void);
bool motion_paint_stroke(const stroke_t *s, const brush_t *b);

/* calibration helper: straight line at constant depth, canvas mm */
bool motion_paint_line(float x0, float y0, float x1, float y1, float depth, const brush_t *b);

/* total stroke-path length emitted so far, mm (stats) */
float motion_paint_distance(void);

#endif
