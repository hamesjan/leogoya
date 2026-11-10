#include "planning/brush_model.h"
#include "config.h"
#include "common/util.h"
#include <math.h>

void brush_defaults(brush_set_t *brushes)
{
    /* first guesses for: 1" flat, 1/2" filbert, #6 round */
    static const brush_t k_defaults[NUM_BRUSHES] = {
        { 10.0f, 4.0f, PAINT_CAPACITY_MM2 * 1.6f, 3.0f, Z_TOUCH_MM },
        { 5.0f, 2.0f, PAINT_CAPACITY_MM2, 3.0f, Z_TOUCH_MM },
        { 1.5f, 1.0f, PAINT_CAPACITY_MM2 * 0.4f, 2.5f, Z_TOUCH_MM },
    };
    for (int i = 0; i < NUM_BRUSHES; i++)
    {
        brushes->brush[i] = k_defaults[i];
    }
}

float brush_depth_for_width(const brush_t *brush, float width_mm)
{
    if (brush->width_per_depth <= 1e-3f)
    {
        return 0;
    }
    return clampf((width_mm - brush->width_at_touch_mm) / brush->width_per_depth, 0.0f, Z_PRESS_MAX_MM);
}

float brush_opacity(const brush_t *brush, float used_mm2)
{
    float used_fraction = clampf(used_mm2 / brush->capacity_mm2, 0.0f, 1.0f);
    return 1.0f - powf(used_fraction, brush->dryness_exponent);
}

float brush_fit(brush_t *brush, const float *depth_mm, const float *width_mm, int count)
{
    double sum_depth = 0;
    double sum_width = 0;
    double sum_depth_sq = 0;
    double sum_depth_width = 0;
    for (int i = 0; i < count; i++)
    {
        sum_depth += depth_mm[i];
        sum_width += width_mm[i];
        sum_depth_sq += depth_mm[i] * depth_mm[i];
        sum_depth_width += depth_mm[i] * width_mm[i];
    }
    double denominator = count * sum_depth_sq - sum_depth * sum_depth;
    if (count < 2 || fabs(denominator) < 1e-9)
    {
        return 0;
    }
    double slope = (count * sum_depth_width - sum_depth * sum_width) / denominator;
    double intercept = (sum_width - slope * sum_depth) / count;

    /* R^2 = 1 - residual sum of squares / total sum of squares */
    double mean_width = sum_width / count;
    double residual_ss = 0;
    double total_ss = 0;
    for (int i = 0; i < count; i++)
    {
        double residual = width_mm[i] - (intercept + slope * depth_mm[i]);
        double deviation = width_mm[i] - mean_width;
        residual_ss += residual * residual;
        total_ss += deviation * deviation;
    }
    if (slope > 0.05) /* reject nonsense fits */
    {
        brush->width_per_depth = (float)slope;
        brush->width_at_touch_mm = (float)intercept;
    }
    return total_ss > 0 ? (float)(1.0 - residual_ss / total_ss) : 0;
}

float brush_measure_width(const lab_image_t *canvas, lab_t background, int row_px, int first_col,
                          int last_col)
{
    const float painted_threshold = 20.0f; /* deltaE from background that counts as "painted" */
    int search_rows = (int)(40.0f / MM_PER_PX);
    double total_painted = 0;
    int painted_cols = 0;
    for (int x = first_col; x <= last_col; x++)
    {
        int painted_rows = 0;
        for (int y = row_px - search_rows; y <= row_px + search_rows; y++)
        {
            if (y < 0 || y >= canvas->height)
            {
                continue;
            }
            if (lab_dist(img_get(canvas, x, y), background) > painted_threshold)
            {
                painted_rows++;
            }
        }
        if (painted_rows)
        {
            total_painted += painted_rows;
            painted_cols++;
        }
    }
    return painted_cols ? (float)(total_painted / painted_cols) * MM_PER_PX : 0;
}
