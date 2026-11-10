#include "planning/brush_model.h"
#include "config.h"
#include "common/util.h"
#include <math.h>


void brush_defaults(brush_set_t *s) {
    /* first guesses for: 1" flat, 1/2" filbert, #6 round */
    static const brush_t d[NUM_BRUSHES] = {
        { 10.0f, 4.0f, PAINT_CAPACITY_MM2 * 1.6f, 3.0f, Z_TOUCH_MM },
        {  5.0f, 2.0f, PAINT_CAPACITY_MM2,        3.0f, Z_TOUCH_MM },
        {  1.5f, 1.0f, PAINT_CAPACITY_MM2 * 0.4f, 2.5f, Z_TOUCH_MM },
    };
    s->magic = BRUSH_MAGIC;
    for (int i = 0; i < NUM_BRUSHES; i++) s->b[i] = d[i];
}

float brush_depth_for_width(const brush_t *b, float w) {
    if (b->k <= 1e-3f) return 0;
    return clampf((w - b->w0) / b->k, 0.0f, Z_PRESS_MAX_MM);
}

float brush_width_for_depth(const brush_t *b, float d) {
    return b->w0 + b->k * clampf(d, 0.0f, Z_PRESS_MAX_MM);
}

float brush_opacity(const brush_t *b, float used) {
    float f = clampf(used / b->capacity_mm2, 0.0f, 1.0f);
    return 1.0f - powf(f, b->dry_exp);
}

float brush_fit(brush_t *b, const float *x, const float *y, int n) {
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < n; i++) { sx += x[i]; sy += y[i]; sxx += x[i] * x[i]; sxy += x[i] * y[i]; }
    double den = n * sxx - sx * sx;
    if (n < 2 || fabs(den) < 1e-9) return 0;
    double k = (n * sxy - sx * sy) / den, w0 = (sy - k * sx) / n;
    double mean = sy / n, ssr = 0, sst = 0;
    for (int i = 0; i < n; i++) {
        double e = y[i] - (w0 + k * x[i]), m = y[i] - mean;
        ssr += e * e; sst += m * m;
    }
    if (k > 0.05) { b->k = (float)k; b->w0 = (float)w0; }   /* reject nonsense fits */
    return sst > 0 ? (float)(1.0 - ssr / sst) : 0;
}

float brush_measure_width(const lab_image_t *c, lab_t bg, int row, int x0, int x1) {
    const float thresh = 20.0f;   /* deltaE that counts as "painted" */
    int search = (int)(40.0f / MM_PER_PX);
    double total = 0; int cols = 0;
    for (int x = x0; x <= x1; x++) {
        int n = 0;
        for (int y = row - search; y <= row + search; y++) {
            if (y < 0 || y >= c->h) continue;
            if (lab_dist(img_get(c, x, y), bg) > thresh) n++;
        }
        if (n) { total += n; cols++; }
    }
    return cols ? (float)(total / cols) * MM_PER_PX : 0;
}
