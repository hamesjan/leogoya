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

typedef struct {
    lab_image_t ref;      /* rectified reference (the "one picture") */
    lab_image_t refb;     /* blurred reference, current layer */
    lab_image_t sim;      /* what we think the canvas looks like */
    float *tmp;           /* blur scratch */
    float *cover;         /* per-stroke alpha mask */
    palette_t pal;
    const brush_set_t *brushes;
    rng_t rng;
    float paint_used;     /* mm^2 since last dip, mirrored by the machine */
    int cur_color;        /* -1 = brush clean */
} planner_t;

typedef struct {
    int candidates, accepted;
    float err_before, err_after;  /* mean deltaE vs sharp reference */
} layer_stats_t;

bool  planner_init(planner_t *p, const palette_t *pal, const brush_set_t *b, uint32_t seed);
void  planner_free(planner_t *p);
void  planner_set_canvas(planner_t *p, const lab_image_t *photo);
float planner_error(const planner_t *p);

/* Fills out[] in paint order, updates p->sim. Returns stroke count. */
int planner_layer(planner_t *p, int layer, stroke_t *out, int max, layer_stats_t *st);

/* Brush is rinsed between layers (brush swap); resets paint state. */
void planner_brush_cleaned(planner_t *p);

#endif
