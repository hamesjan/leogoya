#include "planning/planner.h"
#include "config.h"
#include "hal/hal.h"
#include <float.h>
#include <math.h>
#include <string.h>

static const float k_layer_w[NUM_LAYERS]   = LAYER_WIDTHS_MM;
static const int   k_layer_b[NUM_LAYERS]   = LAYER_BRUSH_ID;
static const float k_layer_thr[NUM_LAYERS] = LAYER_ERR_THRESH;

/* candidate stroke seed */
typedef struct { uint16_t x, y; uint8_t color; } seed_t;
static seed_t *s_seeds;
static int s_seed_cap;

bool planner_init(planner_t *p, const palette_t *pal, const brush_set_t *b, uint32_t seed) {
    memset(p, 0, sizeof *p);
    size_t n = (size_t)WORK_RES * WORK_RES;
    if (!img_alloc(&p->ref, WORK_RES, WORK_RES) || !img_alloc(&p->refb, WORK_RES, WORK_RES) ||
        !img_alloc(&p->sim, WORK_RES, WORK_RES)) return false;
    p->tmp = hal_alloc_large(n * sizeof(float));
    p->cover = hal_alloc_large(n * sizeof(float));
    s_seed_cap = (int)n;
    s_seeds = hal_alloc_large(n * sizeof(seed_t));
    if (!p->tmp || !p->cover || !s_seeds) return false;
    memset(p->cover, 0, n * sizeof(float));
    p->pal = *pal;
    p->brushes = b;
    p->rng.s = seed;
    p->cur_color = -1;
    img_fill(&p->sim, (lab_t){ 95.0f, 0.0f, 2.0f });   /* white gesso */
    return true;
}

void planner_free(planner_t *p) {
    img_free(&p->ref); img_free(&p->refb); img_free(&p->sim);
    hal_free_large(p->tmp); hal_free_large(p->cover); hal_free_large(s_seeds);
    s_seeds = 0;
}

void planner_set_canvas(planner_t *p, const lab_image_t *photo) { img_copy(&p->sim, photo); }

void planner_brush_cleaned(planner_t *p) { p->cur_color = -1; p->paint_used = 0; }

float planner_error(const planner_t *p) {
    double e = 0;
    int n = p->ref.w * p->ref.h;
    for (int i = 0; i < n; i++) {
        lab_t a = { p->ref.L[i], p->ref.a[i], p->ref.b[i] }, s = { p->sim.L[i], p->sim.a[i], p->sim.b[i] };
        e += lab_dist(a, s);
    }
    return (float)(e / n);
}

/* ---- stroke growth (Hertzmann's makeSplineStroke, with a flat-area fallback) ---- */
static int grow_stroke(planner_t *p, int x0, int y0, lab_t col, float step, float default_ang, spt_t *pts) {
    float x = x0 + 0.5f, y = y0 + 0.5f, ldx = 0, ldy = 0;
    int n = 0;
    pts[n++] = (spt_t){ x, y };
    while (n < STROKE_MAX_PTS) {
        int ix = (int)x, iy = (int)y;
        lab_t rb = img_get(&p->refb, ix, iy), sm = img_get(&p->sim, ix, iy);
        /* stop once painting this color here would be worse than leaving it */
        if (n >= STROKE_MIN_PTS && lab_dist(rb, sm) < lab_dist(rb, col)) break;

        float gx, gy;
        img_gradient_L(&p->refb, ix, iy, &gx, &gy);
        float gm = sqrtf(gx * gx + gy * gy), dx, dy;
        if (gm > 1.0f) {               /* follow the edge: perpendicular to gradient */
            dx = -gy / gm; dy = gx / gm;
        } else if (ldx != 0 || ldy != 0) {
            dx = ldx; dy = ldy;        /* flat region: keep going straight */
        } else {
            dx = cosf(default_ang); dy = sinf(default_ang);
        }
        if (dx * ldx + dy * ldy < 0) { dx = -dx; dy = -dy; }
        if (n > 1) {                    /* limit curvature */
            dx = STROKE_CURVATURE * dx + (1 - STROKE_CURVATURE) * ldx;
            dy = STROKE_CURVATURE * dy + (1 - STROKE_CURVATURE) * ldy;
            float m = sqrtf(dx * dx + dy * dy);
            if (m < 1e-4f) break;
            dx /= m; dy /= m;
        }
        float nx = x + step * dx, ny = y + step * dy;
        if (nx < 0 || ny < 0 || nx >= p->ref.w || ny >= p->ref.h) break;
        x = nx; y = ny; ldx = dx; ldy = dy;
        pts[n++] = (spt_t){ x, y };
    }
    return n;
}

/* ---- rasterize stroke into p->cover; returns bbox and covered area ---- */
typedef struct { int x0, y0, x1, y1; } bbox_t;

static float stroke_length_px(const spt_t *pts, int n) {
    float L = 0;
    for (int i = 1; i < n; i++) L += hypotf(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    return L;
}

static void stamp(planner_t *p, float cx, float cy, float r, float alpha, bbox_t *bb) {
    int w = p->ref.w, h = p->ref.h;
    int x0 = (int)floorf(cx - r - 1), x1 = (int)ceilf(cx + r + 1);
    int y0 = (int)floorf(cy - r - 1), y1 = (int)ceilf(cy + r + 1);
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= w) x1 = w - 1;
    if (y1 >= h) y1 = h - 1;
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++) {
            float d = hypotf(x + 0.5f - cx, y + 0.5f - cy);
            float c = clampf((r + BRUSH_SOFT_EDGE_PX - d) / (2 * BRUSH_SOFT_EDGE_PX), 0, 1) * alpha;
            float *cv = &p->cover[y * w + x];
            if (c > *cv) *cv = c;
        }
    if (x0 < bb->x0) bb->x0 = x0;
    if (y0 < bb->y0) bb->y0 = y0;
    if (x1 > bb->x1) bb->x1 = x1;
    if (y1 > bb->y1) bb->y1 = y1;
}

static void rasterize(planner_t *p, const spt_t *pts, int n, float r, float alpha, bbox_t *bb) {
    bb->x0 = bb->y0 = 1 << 30; bb->x1 = bb->y1 = -1;
    stamp(p, pts[0].x, pts[0].y, r, alpha, bb);
    for (int i = 1; i < n; i++) {
        float dx = pts[i].x - pts[i - 1].x, dy = pts[i].y - pts[i - 1].y;
        int steps = (int)ceilf(hypotf(dx, dy) / 0.5f);
        for (int s = 1; s <= steps; s++) {
            float t = (float)s / steps;
            stamp(p, pts[i - 1].x + dx * t, pts[i - 1].y + dy * t, r, alpha, bb);
        }
    }
}

/* Error change (vs blurred ref) if the stroke in p->cover were painted. */
static float score(const planner_t *p, const bbox_t *bb, lab_t col, int *npix) {
    double delta = 0;
    int w = p->ref.w, cnt = 0;
    for (int y = bb->y0; y <= bb->y1; y++)
        for (int x = bb->x0; x <= bb->x1; x++) {
            int i = y * w + x;
            float a = p->cover[i];
            if (a <= 0) continue;
            lab_t r = { p->refb.L[i], p->refb.a[i], p->refb.b[i] };
            lab_t s = { p->sim.L[i], p->sim.a[i], p->sim.b[i] };
            lab_t ns = { s.L + (col.L - s.L) * a, s.a + (col.a - s.a) * a, s.b + (col.b - s.b) * a };
            delta += lab_dist(ns, r) - lab_dist(s, r);
            cnt++;
        }
    *npix = cnt;
    return (float)delta;
}

static void commit_or_clear(planner_t *p, const bbox_t *bb, lab_t col, bool commit) {
    int w = p->ref.w;
    for (int y = bb->y0; y <= bb->y1; y++)
        for (int x = bb->x0; x <= bb->x1; x++) {
            int i = y * w + x;
            float a = p->cover[i];
            if (a <= 0) continue;
            if (commit) {
                p->sim.L[i] += (col.L - p->sim.L[i]) * a;
                p->sim.a[i] += (col.a - p->sim.a[i]) * a;
                p->sim.b[i] += (col.b - p->sim.b[i]) * a;
            }
            p->cover[i] = 0;
        }
}

static float cell_error(const planner_t *p, int cx, int cy, int g, int *wx, int *wy) {
    double e = 0; float worst = -1; int n = 0;
    for (int y = cy; y < cy + g && y < p->ref.h; y++)
        for (int x = cx; x < cx + g && x < p->ref.w; x++) {
            float d = lab_dist(img_get(&p->refb, x, y), img_get(&p->sim, x, y));
            e += d; n++;
            if (d > worst) { worst = d; *wx = x; *wy = y; }
        }
    return n ? (float)(e / n) : 0;
}

int planner_layer(planner_t *p, int layer, stroke_t *out, int max, layer_stats_t *st) {
    const float width = k_layer_w[layer];
    const int brush_id = k_layer_b[layer];
    const brush_t *br = &p->brushes->b[brush_id];
    const float R = 0.5f * width / MM_PER_PX;          /* brush radius, px */
    const float thr = k_layer_thr[layer];
    const float default_ang = rng_unit(&p->rng) * 3.14159265f;

    memset(st, 0, sizeof *st);
    st->err_before = planner_error(p);
    img_blur(&p->refb, &p->ref, BLUR_FACTOR * R * 2.0f, p->tmp);

    /* 1) seeds: worst pixel of every grid cell above threshold */
    int g = (int)(2 * R); if (g < 3) g = 3;          /* >= ~11 mm cells */
    const float step = R > 2.0f ? R : 2.0f;
    int ns = 0;
    for (int cy = 0; cy < p->ref.h; cy += g)
        for (int cx = 0; cx < p->ref.w; cx += g) {
            int wx = cx, wy = cy;
            if (cell_error(p, cx, cy, g, &wx, &wy) < thr || ns >= s_seed_cap) continue;
            /* jitter so strokes don't line up on the grid */
            wx = (int)clampf(wx + (rng_unit(&p->rng) - 0.5f) * g * 0.5f, 0, p->ref.w - 1);
            wy = (int)clampf(wy + (rng_unit(&p->rng) - 0.5f) * g * 0.5f, 0, p->ref.h - 1);
            s_seeds[ns++] = (seed_t){ (uint16_t)wx, (uint16_t)wy,
                                      (uint8_t)palette_nearest(&p->pal, img_get(&p->refb, wx, wy)) };
        }
    st->candidates = ns;

    /* 2) order: by color (palette is sorted dark->light), then greedy nearest neighbour */
    int count = 0;
    for (int color = 0; color < p->pal.n && count < max; color++) {
        float px = 0, py = 0;
        for (;;) {
            int best = -1; float bd = FLT_MAX;
            for (int i = 0; i < ns; i++) {
                if (s_seeds[i].color != color) continue;
                float d = (s_seeds[i].x - px) * (s_seeds[i].x - px) + (s_seeds[i].y - py) * (s_seeds[i].y - py);
                if (d < bd) { bd = d; best = i; }
            }
            if (best < 0 || count >= max) break;
            seed_t sd = s_seeds[best];
            s_seeds[best].color = 0xFF;                 /* consumed */

            /* re-check: earlier strokes this layer may already cover it */
            int wx = sd.x, wy = sd.y;
            if (cell_error(p, sd.x - g / 2 < 0 ? 0 : sd.x - g / 2, sd.y - g / 2 < 0 ? 0 : sd.y - g / 2, g, &wx, &wy) < thr)
                continue;

            lab_t col = p->pal.c[color];
            stroke_t *s = &out[count];
            s->npts = (uint8_t)grow_stroke(p, sd.x, sd.y, col, step, default_ang, s->p);

            /* paint load bookkeeping -> decides dips, sets opacity in sim */
            float area = (stroke_length_px(s->p, s->npts) + 2 * R) * MM_PER_PX * width;
            uint8_t reload = 0;
            if (p->cur_color != color) reload = p->cur_color < 0 ? 1 : 2;   /* rinse only if dirty */
            else if (p->paint_used + area > RELOAD_FRACTION * br->capacity_mm2) reload = 1;
            float used = reload ? 0 : p->paint_used;
            float alpha = STROKE_OPACITY * brush_opacity(br, used + 0.5f * area);
            if (alpha < 0.3f) alpha = 0.3f;

            bbox_t bb;
            rasterize(p, s->p, s->npts, R, alpha, &bb);
            int npix;
            float d = score(p, &bb, col, &npix);
            bool keep = npix > 0 && d < -1.0f * npix;    /* >1 deltaE mean improvement */
            commit_or_clear(p, &bb, col, keep);
            if (!keep) continue;

            s->color = (uint8_t)color;
            s->brush = (uint8_t)brush_id;
            s->width_mm = width;
            s->reload = reload;
            p->cur_color = color;
            p->paint_used = used + area;
            px = s->p[s->npts - 1].x; py = s->p[s->npts - 1].y;
            count++;
        }
    }
    st->accepted = count;
    st->err_after = planner_error(p);
    return count;
}
