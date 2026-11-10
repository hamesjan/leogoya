/*
 * planner.h - stroke-based rendering with greedy, simulator-verified acceptance.
 *
 * Per layer (big brush -> small brush):
 *   1. blur the reference to the brush scale                (Hertzmann 1998)
 *   2. on a grid, find cells where canvas differs from it   (error map)
 *   3. grow a curved stroke from each cell's worst pixel, following the
 *      direction *perpendicular* to the luminance gradient (i.e. along edges)
 *   4. render each candidate into a simulated canvas and keep it only if it
 *      lowers the deltaE error                              (greedy optimizer)
 *   5. order strokes by color (fewest rinses) then nearest-neighbour (least travel)
 *
 * With CLOSED_LOOP the simulated canvas is replaced by a photo of the real one
 * before each layer, so the next layer corrects what the paint actually did.
 */
#ifndef LEOGOYA_PLANNER_H
#define LEOGOYA_PLANNER_H

#include "planning/brush_model.h"
#include "vision/image.h"
#include "vision/palette.h"
#include "planning/stroke.h"
#include "common/util.h"

typedef struct
{
    lab_image_t reference;         /* rectified reference image */
    lab_image_t reference_blurred; /* reference blurred to the current brush size */
    lab_image_t canvas_sim;        /* what we think the canvas looks like */
    float *blur_scratch;           /* width*height scratch for img_blur */
    float *stroke_coverage;        /* per-pixel alpha of the stroke being evaluated */
    palette_t palette;
    const brush_set_t *brushes;
    rng_t rng;
    float paint_used_mm2; /* canvas area painted since the last dip */
    int loaded_color;     /* palette index on the brush, -1 = brush clean */
} planner_t;

typedef struct
{
    int candidate_count;
    int accepted_count;
    float error_before; /* mean deltaE vs the sharp reference */
    float error_after;
} layer_stats_t;

bool planner_init(planner_t *planner, const palette_t *palette, const brush_set_t *brushes, uint32_t seed);
void planner_free(planner_t *planner);
void planner_set_canvas(planner_t *planner, const lab_image_t *photo);
float planner_error(const planner_t *planner);

/* Fills strokes[] in paint order and updates canvas_sim. Returns the stroke count. */
int planner_layer(planner_t *planner, int layer, stroke_t *strokes, int max_strokes, layer_stats_t *stats);

/* Brush is rinsed between layers (brush swap); resets paint state. */
void planner_brush_cleaned(planner_t *planner);

#endif
