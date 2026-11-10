/*
 * brush_model.h - learned model of how a physical brush behaves.
 *
 *   width_mm(depth)   = w0 + k * depth          (linear, fit by least squares)
 *   opacity(used_mm2) = 1 - (used/capacity)^dry (paint runs out along a load)
 *
 * The robot learns w0, k and capacity on-device: it paints a calibration strip
 * at known press depths, photographs it, measures the widths, and fits a line.
 * This is the piece that closes the gap between "planned stroke" and "paint".
 */
#ifndef LEOGOYA_BRUSH_MODEL_H
#define LEOGOYA_BRUSH_MODEL_H

#include "vision/image.h"
#include <stdint.h>

#define NUM_BRUSHES 3
#define BRUSH_MAGIC 0xB2054001u

typedef struct {
    float w0, k;          /* mm, mm per mm of press depth */
    float capacity_mm2;   /* area one dip covers */
    float dry_exp;        /* how abruptly it goes dry */
    float z_touch;        /* machine Z where bristles touch canvas */
} brush_t;

typedef struct {
    uint32_t magic;
    brush_t b[NUM_BRUSHES];
} brush_set_t;

void  brush_defaults(brush_set_t *s);
float brush_depth_for_width(const brush_t *b, float width_mm);
float brush_width_for_depth(const brush_t *b, float depth_mm);
float brush_opacity(const brush_t *b, float used_mm2);

/* least-squares line fit, returns R^2 */
float brush_fit(brush_t *b, const float *depth, const float *width, int n);

/* Measure the painted width of a horizontal calibration line centered on
 * canvas row `row_px`, between columns x0..x1, against background color bg. */
float brush_measure_width(const lab_image_t *canvas, lab_t bg, int row_px, int x0, int x1);

#endif
