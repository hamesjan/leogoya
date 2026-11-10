/*
 * brush_model.h - learned model of how a physical brush behaves.
 *
 *   width_mm(depth)   = width_at_touch_mm + width_per_depth * depth   (linear, least squares)
 *   opacity(used_mm2) = 1 - (used / capacity) ^ dryness_exponent      (paint runs out along a load)
 *
 * Calibration paints a strip at known press depths, photographs it, measures
 * the widths, and fits the line. This closes the gap between "planned stroke"
 * and "paint".
 */
#ifndef LEOGOYA_BRUSH_MODEL_H
#define LEOGOYA_BRUSH_MODEL_H

#include "vision/image.h"
#include <stdint.h>

#define NUM_BRUSHES 3

typedef struct
{
    float width_at_touch_mm; /* stroke width when the bristles just touch */
    float width_per_depth;   /* extra mm of width per mm of press depth */
    float capacity_mm2;      /* canvas area one dip covers */
    float dryness_exponent;  /* how abruptly the brush goes dry */
    float z_touch;           /* machine Z where the bristles touch the canvas */
} brush_t;

typedef struct
{
    brush_t brush[NUM_BRUSHES];
} brush_set_t;

void brush_defaults(brush_set_t *brushes);
float brush_depth_for_width(const brush_t *brush, float width_mm);
float brush_opacity(const brush_t *brush, float used_mm2);

/* Least-squares line fit of width vs depth; updates the brush, returns R^2. */
float brush_fit(brush_t *brush, const float *depth_mm, const float *width_mm, int count);

/* Measure the painted width of a horizontal calibration line centered on
 * canvas row `row_px`, between columns first_col..last_col, against the
 * unpainted background color. */
float brush_measure_width(const lab_image_t *canvas, lab_t background, int row_px, int first_col,
                          int last_col);

#endif
