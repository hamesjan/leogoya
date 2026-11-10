#include "planning/planner.h"
#include "config.h"
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <string.h>

static const float k_layer_width_mm[NUM_LAYERS] = LAYER_WIDTHS_MM;
static const int k_layer_brush_id[NUM_LAYERS] = LAYER_BRUSH_ID;
static const float k_layer_error_threshold[NUM_LAYERS] = LAYER_ERR_THRESH;

#define SEED_CONSUMED 0xFF /* seed_t.color once the seed has been turned into a stroke */

/* A candidate stroke start: grid cell's worst pixel and the palette color it needs. */
typedef struct
{
    uint16_t x;
    uint16_t y;
    uint8_t color;
} seed_t;

static seed_t *s_seeds;
static int s_seed_capacity;

/* Pixel bounds touched by a stroke (inclusive). */
typedef struct
{
    int min_x;
    int min_y;
    int max_x;
    int max_y;
} bbox_t;

bool planner_init(planner_t *planner, const palette_t *palette, const brush_set_t *brushes, uint32_t seed)
{
    memset(planner, 0, sizeof *planner);
    size_t pixel_count = (size_t)WORK_RES * WORK_RES;
    if (!img_alloc(&planner->reference, WORK_RES, WORK_RES) ||
        !img_alloc(&planner->reference_blurred, WORK_RES, WORK_RES) ||
        !img_alloc(&planner->canvas_sim, WORK_RES, WORK_RES))
    {
        return false;
    }
    planner->blur_scratch = malloc(pixel_count * sizeof(float));
    planner->stroke_coverage = malloc(pixel_count * sizeof(float));
    s_seed_capacity = (int)pixel_count;
    s_seeds = malloc(pixel_count * sizeof(seed_t));
    if (!planner->blur_scratch || !planner->stroke_coverage || !s_seeds)
    {
        return false;
    }
    memset(planner->stroke_coverage, 0, pixel_count * sizeof(float));
    planner->palette = *palette;
    planner->brushes = brushes;
    planner->rng.state = seed;
    planner->loaded_color = -1;
    img_fill(&planner->canvas_sim, (lab_t){ 95.0f, 0.0f, 2.0f }); /* white gesso */
    return true;
}

void planner_free(planner_t *planner)
{
    img_free(&planner->reference);
    img_free(&planner->reference_blurred);
    img_free(&planner->canvas_sim);
    free(planner->blur_scratch);
    free(planner->stroke_coverage);
    free(s_seeds);
    s_seeds = 0;
}

void planner_set_canvas(planner_t *planner, const lab_image_t *photo)
{
    img_copy(&planner->canvas_sim, photo);
}

void planner_brush_cleaned(planner_t *planner)
{
    planner->loaded_color = -1;
    planner->paint_used_mm2 = 0;
}

float planner_error(const planner_t *planner)
{
    const lab_image_t *reference = &planner->reference;
    const lab_image_t *canvas = &planner->canvas_sim;
    double total_error = 0;
    int pixel_count = reference->width * reference->height;
    for (int i = 0; i < pixel_count; i++)
    {
        lab_t wanted = { reference->L[i], reference->a[i], reference->b[i] };
        lab_t painted = { canvas->L[i], canvas->a[i], canvas->b[i] };
        total_error += lab_dist(wanted, painted);
    }
    return (float)(total_error / pixel_count);
}

/* ---- stroke growth (Hertzmann's makeSplineStroke, with a flat-area fallback) ---- */
static int grow_stroke(planner_t *planner, int start_x, int start_y, lab_t color, float step_px,
                       float fallback_angle, stroke_point_t *points)
{
    float x = start_x + 0.5f;
    float y = start_y + 0.5f;
    float prev_dir_x = 0;
    float prev_dir_y = 0;
    int num_points = 0;
    points[num_points++] = (stroke_point_t){ x, y };
    while (num_points < STROKE_MAX_PTS)
    {
        int pixel_x = (int)x;
        int pixel_y = (int)y;
        lab_t wanted = img_get(&planner->reference_blurred, pixel_x, pixel_y);
        lab_t painted = img_get(&planner->canvas_sim, pixel_x, pixel_y);
        /* stop once painting this color here would be worse than leaving it */
        if (num_points >= STROKE_MIN_PTS && lab_dist(wanted, painted) < lab_dist(wanted, color))
        {
            break;
        }

        float grad_x;
        float grad_y;
        img_gradient_L(&planner->reference_blurred, pixel_x, pixel_y, &grad_x, &grad_y);
        float grad_magnitude = sqrtf(grad_x * grad_x + grad_y * grad_y);
        float dir_x;
        float dir_y;
        if (grad_magnitude > 1.0f)
        {
            /* follow the edge: perpendicular to the gradient */
            dir_x = -grad_y / grad_magnitude;
            dir_y = grad_x / grad_magnitude;
        }
        else if (prev_dir_x != 0 || prev_dir_y != 0)
        {
            /* flat region: keep going straight */
            dir_x = prev_dir_x;
            dir_y = prev_dir_y;
        }
        else
        {
            dir_x = cosf(fallback_angle);
            dir_y = sinf(fallback_angle);
        }
        if (dir_x * prev_dir_x + dir_y * prev_dir_y < 0)
        {
            dir_x = -dir_x;
            dir_y = -dir_y;
        }
        if (num_points > 1)
        {
            /* limit curvature: blend with the previous direction */
            dir_x = STROKE_CURVATURE * dir_x + (1 - STROKE_CURVATURE) * prev_dir_x;
            dir_y = STROKE_CURVATURE * dir_y + (1 - STROKE_CURVATURE) * prev_dir_y;
            float dir_length = sqrtf(dir_x * dir_x + dir_y * dir_y);
            if (dir_length < 1e-4f)
            {
                break;
            }
            dir_x /= dir_length;
            dir_y /= dir_length;
        }
        float next_x = x + step_px * dir_x;
        float next_y = y + step_px * dir_y;
        if (next_x < 0 || next_y < 0 || next_x >= planner->reference.width ||
            next_y >= planner->reference.height)
        {
            break;
        }
        x = next_x;
        y = next_y;
        prev_dir_x = dir_x;
        prev_dir_y = dir_y;
        points[num_points++] = (stroke_point_t){ x, y };
    }
    return num_points;
}

/* ---- rasterize a stroke into planner->stroke_coverage ---- */

static float stroke_length_px(const stroke_point_t *points, int num_points)
{
    float length = 0;
    for (int i = 1; i < num_points; i++)
    {
        length += hypotf(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    }
    return length;
}

/* Paint one soft round brush dab; coverage keeps the max alpha per pixel. */
static void stamp(planner_t *planner, float center_x, float center_y, float radius, float alpha, bbox_t *bbox)
{
    int width = planner->reference.width;
    int height = planner->reference.height;
    int min_x = (int)floorf(center_x - radius - 1);
    int max_x = (int)ceilf(center_x + radius + 1);
    int min_y = (int)floorf(center_y - radius - 1);
    int max_y = (int)ceilf(center_y + radius + 1);
    if (min_x < 0)
    {
        min_x = 0;
    }
    if (min_y < 0)
    {
        min_y = 0;
    }
    if (max_x >= width)
    {
        max_x = width - 1;
    }
    if (max_y >= height)
    {
        max_y = height - 1;
    }
    for (int y = min_y; y <= max_y; y++)
    {
        for (int x = min_x; x <= max_x; x++)
        {
            float distance = hypotf(x + 0.5f - center_x, y + 0.5f - center_y);
            float edge = clampf((radius + BRUSH_SOFT_EDGE_PX - distance) / (2 * BRUSH_SOFT_EDGE_PX), 0, 1);
            float dab_alpha = edge * alpha;
            float *coverage = &planner->stroke_coverage[y * width + x];
            if (dab_alpha > *coverage)
            {
                *coverage = dab_alpha;
            }
        }
    }
    if (min_x < bbox->min_x)
    {
        bbox->min_x = min_x;
    }
    if (min_y < bbox->min_y)
    {
        bbox->min_y = min_y;
    }
    if (max_x > bbox->max_x)
    {
        bbox->max_x = max_x;
    }
    if (max_y > bbox->max_y)
    {
        bbox->max_y = max_y;
    }
}

static void rasterize(planner_t *planner, const stroke_point_t *points, int num_points, float radius,
                      float alpha, bbox_t *bbox)
{
    bbox->min_x = 1 << 30;
    bbox->min_y = 1 << 30;
    bbox->max_x = -1;
    bbox->max_y = -1;
    stamp(planner, points[0].x, points[0].y, radius, alpha, bbox);
    for (int i = 1; i < num_points; i++)
    {
        float seg_x = points[i].x - points[i - 1].x;
        float seg_y = points[i].y - points[i - 1].y;
        int dab_count = (int)ceilf(hypotf(seg_x, seg_y) / 0.5f); /* a dab every half pixel */
        for (int dab = 1; dab <= dab_count; dab++)
        {
            float t = (float)dab / dab_count;
            stamp(planner, points[i - 1].x + seg_x * t, points[i - 1].y + seg_y * t, radius, alpha, bbox);
        }
    }
}

/* Change in error (vs the blurred reference) if the stroke in stroke_coverage were
 * painted. Negative = improvement. */
static float score(const planner_t *planner, const bbox_t *bbox, lab_t color, int *covered_pixels)
{
    const lab_image_t *target = &planner->reference_blurred;
    const lab_image_t *canvas = &planner->canvas_sim;
    double error_change = 0;
    int width = planner->reference.width;
    int covered = 0;
    for (int y = bbox->min_y; y <= bbox->max_y; y++)
    {
        for (int x = bbox->min_x; x <= bbox->max_x; x++)
        {
            int i = y * width + x;
            float alpha = planner->stroke_coverage[i];
            if (alpha <= 0)
            {
                continue;
            }
            lab_t wanted = { target->L[i], target->a[i], target->b[i] };
            lab_t before = { canvas->L[i], canvas->a[i], canvas->b[i] };
            lab_t after = { before.L + (color.L - before.L) * alpha, before.a + (color.a - before.a) * alpha,
                            before.b + (color.b - before.b) * alpha };
            error_change += lab_dist(after, wanted) - lab_dist(before, wanted);
            covered++;
        }
    }
    *covered_pixels = covered;
    return (float)error_change;
}

/* Blend the evaluated stroke into canvas_sim (if commit), then clear its coverage. */
static void commit_or_clear(planner_t *planner, const bbox_t *bbox, lab_t color, bool commit)
{
    lab_image_t *canvas = &planner->canvas_sim;
    int width = planner->reference.width;
    for (int y = bbox->min_y; y <= bbox->max_y; y++)
    {
        for (int x = bbox->min_x; x <= bbox->max_x; x++)
        {
            int i = y * width + x;
            float alpha = planner->stroke_coverage[i];
            if (alpha <= 0)
            {
                continue;
            }
            if (commit)
            {
                canvas->L[i] += (color.L - canvas->L[i]) * alpha;
                canvas->a[i] += (color.a - canvas->a[i]) * alpha;
                canvas->b[i] += (color.b - canvas->b[i]) * alpha;
            }
            planner->stroke_coverage[i] = 0;
        }
    }
}

/* Mean error over a grid cell; also reports the cell's worst pixel. */
static float cell_error(const planner_t *planner, int cell_x, int cell_y, int cell_size, int *worst_x,
                        int *worst_y)
{
    double total_error = 0;
    float worst_error = -1;
    int pixel_count = 0;
    for (int y = cell_y; y < cell_y + cell_size && y < planner->reference.height; y++)
    {
        for (int x = cell_x; x < cell_x + cell_size && x < planner->reference.width; x++)
        {
            float error =
                lab_dist(img_get(&planner->reference_blurred, x, y), img_get(&planner->canvas_sim, x, y));
            total_error += error;
            pixel_count++;
            if (error > worst_error)
            {
                worst_error = error;
                *worst_x = x;
                *worst_y = y;
            }
        }
    }
    return pixel_count ? (float)(total_error / pixel_count) : 0;
}

int planner_layer(planner_t *planner, int layer, stroke_t *strokes, int max_strokes, layer_stats_t *stats)
{
    const float width_mm = k_layer_width_mm[layer];
    const int brush_id = k_layer_brush_id[layer];
    const brush_t *brush = &planner->brushes->brush[brush_id];
    const float radius_px = 0.5f * width_mm / MM_PER_PX;
    const float error_threshold = k_layer_error_threshold[layer];
    const float fallback_angle = rng_unit(&planner->rng) * 3.14159265f;

    memset(stats, 0, sizeof *stats);
    stats->error_before = planner_error(planner);
    img_blur(&planner->reference_blurred, &planner->reference, BLUR_FACTOR * radius_px * 2.0f,
             planner->blur_scratch);

    /* 1) seeds: worst pixel of every grid cell whose error is above the threshold */
    int cell_size = (int)(2 * radius_px);
    if (cell_size < 3)
    {
        cell_size = 3; /* >= ~11 mm cells */
    }
    const float step_px = radius_px > 2.0f ? radius_px : 2.0f;
    int seed_count = 0;
    for (int cell_y = 0; cell_y < planner->reference.height; cell_y += cell_size)
    {
        for (int cell_x = 0; cell_x < planner->reference.width; cell_x += cell_size)
        {
            int worst_x = cell_x;
            int worst_y = cell_y;
            if (cell_error(planner, cell_x, cell_y, cell_size, &worst_x, &worst_y) < error_threshold ||
                seed_count >= s_seed_capacity)
            {
                continue;
            }
            /* jitter so strokes don't line up on the grid */
            worst_x = (int)clampf(worst_x + (rng_unit(&planner->rng) - 0.5f) * cell_size * 0.5f, 0,
                                  planner->reference.width - 1);
            worst_y = (int)clampf(worst_y + (rng_unit(&planner->rng) - 0.5f) * cell_size * 0.5f, 0,
                                  planner->reference.height - 1);
            lab_t wanted = img_get(&planner->reference_blurred, worst_x, worst_y);
            s_seeds[seed_count++] = (seed_t){ (uint16_t)worst_x, (uint16_t)worst_y,
                                              (uint8_t)palette_nearest(&planner->palette, wanted) };
        }
    }
    stats->candidate_count = seed_count;

    /* 2) order: by color (palette is sorted dark->light), then greedy nearest neighbour */
    int stroke_count = 0;
    for (int color = 0; color < planner->palette.count && stroke_count < max_strokes; color++)
    {
        float last_end_x = 0;
        float last_end_y = 0;
        for (;;)
        {
            int nearest = -1;
            float nearest_dist_sq = FLT_MAX;
            for (int i = 0; i < seed_count; i++)
            {
                if (s_seeds[i].color != color)
                {
                    continue;
                }
                float dx = s_seeds[i].x - last_end_x;
                float dy = s_seeds[i].y - last_end_y;
                float dist_sq = dx * dx + dy * dy;
                if (dist_sq < nearest_dist_sq)
                {
                    nearest_dist_sq = dist_sq;
                    nearest = i;
                }
            }
            if (nearest < 0 || stroke_count >= max_strokes)
            {
                break;
            }
            seed_t seed = s_seeds[nearest];
            s_seeds[nearest].color = SEED_CONSUMED;

            /* re-check: earlier strokes in this layer may already cover it */
            int cell_x = seed.x - cell_size / 2 < 0 ? 0 : seed.x - cell_size / 2;
            int cell_y = seed.y - cell_size / 2 < 0 ? 0 : seed.y - cell_size / 2;
            int worst_x = seed.x;
            int worst_y = seed.y;
            if (cell_error(planner, cell_x, cell_y, cell_size, &worst_x, &worst_y) < error_threshold)
            {
                continue;
            }

            lab_t paint_color = planner->palette.colors[color];
            stroke_t *stroke = &strokes[stroke_count];
            stroke->num_points = (uint8_t)grow_stroke(planner, seed.x, seed.y, paint_color, step_px,
                                                      fallback_angle, stroke->points);

            /* paint-load bookkeeping: decides dips, sets the opacity in the simulation */
            float area_mm2 =
                (stroke_length_px(stroke->points, stroke->num_points) + 2 * radius_px) * MM_PER_PX * width_mm;
            reload_t reload = RELOAD_NONE;
            if (planner->loaded_color != color)
            {
                reload = planner->loaded_color < 0 ? RELOAD_DIP : RELOAD_RINSE; /* rinse only if dirty */
            }
            else if (planner->paint_used_mm2 + area_mm2 > RELOAD_FRACTION * brush->capacity_mm2)
            {
                reload = RELOAD_DIP;
            }
            float used_before_mm2 = reload ? 0 : planner->paint_used_mm2;
            float alpha = STROKE_OPACITY * brush_opacity(brush, used_before_mm2 + 0.5f * area_mm2);
            if (alpha < 0.3f)
            {
                alpha = 0.3f;
            }

            bbox_t bbox;
            rasterize(planner, stroke->points, stroke->num_points, radius_px, alpha, &bbox);
            int covered_pixels;
            float error_change = score(planner, &bbox, paint_color, &covered_pixels);
            bool keep = covered_pixels > 0 && error_change < -1.0f * covered_pixels; /* >1 deltaE mean gain */
            commit_or_clear(planner, &bbox, paint_color, keep);
            if (!keep)
            {
                continue;
            }

            stroke->color = (uint8_t)color;
            stroke->brush = (uint8_t)brush_id;
            stroke->width_mm = width_mm;
            stroke->reload = (uint8_t)reload;
            planner->loaded_color = color;
            planner->paint_used_mm2 = used_before_mm2 + area_mm2;
            last_end_x = stroke->points[stroke->num_points - 1].x;
            last_end_y = stroke->points[stroke->num_points - 1].y;
            stroke_count++;
        }
    }
    stats->accepted_count = stroke_count;
    stats->error_after = planner_error(planner);
    return stroke_count;
}
