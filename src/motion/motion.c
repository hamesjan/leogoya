#include "motion/motion.h"
#include "config.h"
#include "hal/hal.h"
#include "common/util.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#define DEG(r) ((r) * 57.29577951f)
#define TIMEOUT_MOVE  30000u
#define TIMEOUT_HOME 120000u

static float s_yaw = 0;          /* current yaw command, degrees, unwrapped */
static float s_paint_dist = 0;

static bool send(const char *fmt, ...) {
    char line[96];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    bool ok = hal_motion_send(line, TIMEOUT_MOVE);
    if (!ok) hal_log("motion: '%s' failed\n", line);
    return ok;
}

/* canvas px -> machine mm */
static inline float mx(float px) { return CANVAS_X0_MM + px * MM_PER_PX; }
static inline float my(float py) { return CANVAS_Y0_MM + py * MM_PER_PX; }

/* A flat brush is symmetric under 180 deg rotation, so pick the equivalent
 * angle nearest the current yaw: minimizes rotation and cable wind-up. */
static float nearest_yaw(float from, float tangent_deg) {
#if BRUSH_USES_YAW
    float a = tangent_deg;
    while (a - from > 90.0f) a -= 180.0f;
    while (a - from < -90.0f) a += 180.0f;
    return a;
#else
    (void)tangent_deg;
    return from;
#endif
}

bool motion_init(void) {
    bool ok = send("G21") && send("G90");
#if YAW_AXIS == 'E'
    ok = ok && send("M302 P1") && send("M82");     /* allow "cold extrusion" = yaw moves */
#endif
    ok = ok && hal_motion_send("G28", TIMEOUT_HOME);
    ok = ok && send("G0 Z%.2f F%d", Z_SAFE_MM, FEED_Z);
    ok = ok && send("G92 %c0", YAW_AXIS);          /* yaw homed by hand: brush flat edge along Y */
    s_yaw = 0;
    return ok;
}

static bool travel(float x, float y) {
    return send("G0 Z%.2f F%d", Z_SAFE_MM, FEED_Z) &&
           send("G0 X%.2f Y%.2f F%d", x, y, FEED_TRAVEL);
}

bool motion_park(void) {
    /* unwind yaw while parked so it never accumulates past a turn */
    s_yaw = fmodf(s_yaw, 180.0f);
    /* Marlin answers "ok" when a move is queued, not done: M400 blocks until
     * the gantry has actually stopped, so the camera never sees it mid-move. */
    return travel(PARK_X_MM, PARK_Y_MM) && send("G0 %c%.2f F%d", YAW_AXIS, s_yaw, FEED_YAW_DEG) &&
           hal_motion_send("M400", TIMEOUT_HOME);
}

static bool swirl(float cx, float cy, float z, float r, int turns) {
    if (!send("G1 Z%.2f F%d", z, FEED_Z)) return false;
    for (int t = 0; t < turns * 8; t++) {
        float a = t * 0.785398f;
        if (!send("G1 X%.2f Y%.2f F%d", cx + r * cosf(a), cy + r * sinf(a), FEED_PAINT)) return false;
    }
    return send("G0 Z%.2f F%d", Z_SAFE_MM, FEED_Z);
}

bool motion_rinse(void) {
    /* water: swirl; sponge: two drags to wick water out of the bristles */
    if (!travel(WATER_X_MM, WATER_Y_MM) || !swirl(WATER_X_MM, WATER_Y_MM, WATER_DIP_Z_MM, 10, 4)) return false;
    for (int i = 0; i < 2; i++) {
        if (!travel(SPONGE_X_MM - 30, SPONGE_Y_MM + i * 12) ||
            !send("G1 Z%.2f F%d", SPONGE_Z_MM, FEED_Z) ||
            !send("G1 X%.2f F%d", SPONGE_X_MM + 30, FEED_PAINT)) return false;
    }
    return send("G0 Z%.2f F%d", Z_SAFE_MM, FEED_Z);
}

bool motion_load_paint(int pot, bool rinse_first) {
    if (rinse_first && !motion_rinse()) return false;
    float cx = POT_X_MM, cy = POT_Y0_MM + pot * POT_PITCH_MM;
    if (!travel(cx, cy) || !swirl(cx, cy, POT_DIP_Z_MM, 6, 2)) return false;
    /* scrape excess on the pot rim: lower to rim height, drag outward */
    return send("G1 Z%.2f F%d", POT_DIP_Z_MM + 8.0f, FEED_Z) &&
           send("G1 X%.2f F%d", cx + POT_RADIUS_MM + 4.0f, FEED_PAINT / 2) &&
           send("G0 Z%.2f F%d", Z_SAFE_MM, FEED_Z);
}

/* ---- stroke trajectory ---- */

#define MAX_PATH 512
typedef struct { float x, y, s, yaw; } path_pt_t;   /* machine mm, arc length, deg */
static path_pt_t s_path[MAX_PATH];

/* Centripetal-ish uniform Catmull-Rom through control points, resampled by length. */
static int build_path(const stroke_t *st) {
    int n = st->npts, m = 0;
    if (n == 1) {                       /* a dab: make a tiny straight stroke */
        s_path[m++] = (path_pt_t){ mx(st->p[0].x) - 1, my(st->p[0].y), 0, 0 };
        s_path[m++] = (path_pt_t){ mx(st->p[0].x) + 1, my(st->p[0].y), 2, 0 };
        return m;
    }
    for (int i = 0; i < n - 1 && m < MAX_PATH - 1; i++) {
        spt_t p0 = st->p[i > 0 ? i - 1 : 0], p1 = st->p[i], p2 = st->p[i + 1];
        spt_t p3 = st->p[i + 2 < n ? i + 2 : n - 1];
        float seg = hypotf(p2.x - p1.x, p2.y - p1.y) * MM_PER_PX;
        int k = (int)ceilf(seg / PATH_STEP_MM);
        if (k < 1) k = 1;
        for (int j = (i == 0 ? 0 : 1); j <= k && m < MAX_PATH; j++) {
            float t = (float)j / k, t2 = t * t, t3 = t2 * t;
            float x = 0.5f * (2 * p1.x + (-p0.x + p2.x) * t + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 +
                              (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3);
            float y = 0.5f * (2 * p1.y + (-p0.y + p2.y) * t + (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 +
                              (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3);
            s_path[m++] = (path_pt_t){ mx(x), my(y), 0, 0 };
        }
    }
    /* arc length + tangent yaw (unwrapped within the stroke, 180-deg symmetric) */
    float yaw = s_yaw;
    for (int i = 0; i < m; i++) {
        int a = i > 0 ? i - 1 : 0, b = i < m - 1 ? i + 1 : m - 1;
        if (i > 0) s_path[i].s = s_path[i - 1].s + hypotf(s_path[i].x - s_path[i - 1].x, s_path[i].y - s_path[i - 1].y);
        float tan_deg = DEG(atan2f(s_path[b].y - s_path[a].y, s_path[b].x - s_path[a].x));
        yaw = nearest_yaw(yaw, tan_deg);
        if (i == 0) {           /* re-center between strokes so yaw never winds up */
            while (yaw > 180.0f) yaw -= 180.0f;
            while (yaw < -180.0f) yaw += 180.0f;
        }
        s_path[i].yaw = yaw;
    }
    return m;
}

/* Press profile: ramp in over LANDING_MM, hold, taper out over LIFTOFF_MM. */
static float depth_at(float s, float total, float body_depth) {
    float in = clampf(s / LANDING_MM, 0, 1);
    float out = clampf((total - s) / LIFTOFF_MM, 0, 1);
    float f = in < out ? in : out;
    return body_depth * (0.35f + 0.65f * f);   /* never fully lose contact mid-stroke */
}

bool motion_paint_stroke(const stroke_t *st, const brush_t *b) {
    int m = build_path(st);
    if (m < 2) return true;
    float total = s_path[m - 1].s;
    float depth = brush_depth_for_width(b, st->width_mm);

    /* approach from behind the start along the initial tangent, brush hovering */
    float ux = s_path[1].x - s_path[0].x, uy = s_path[1].y - s_path[0].y, ul = hypotf(ux, uy);
    if (ul < 1e-3f) { ux = 1; uy = 0; ul = 1; }
    ux /= ul; uy /= ul;
    if (!travel(s_path[0].x - APPROACH_MM * ux, s_path[0].y - APPROACH_MM * uy)) return false;
    if (!send("G0 %c%.2f F%d", YAW_AXIS, s_path[0].yaw, FEED_YAW_DEG)) return false;
    if (!send("G1 Z%.2f F%d", b->z_touch + 1.0f, FEED_Z)) return false;

    /* touch down while already moving: avoids the blob a vertical plunge leaves */
    for (int i = 0; i < m; i++) {
        float z = b->z_touch - depth_at(s_path[i].s, total, depth);
        if (!send("G1 X%.2f Y%.2f Z%.3f %c%.2f F%d", s_path[i].x, s_path[i].y, z, YAW_AXIS,
                  s_path[i].yaw, FEED_PAINT)) return false;
    }
    s_yaw = s_path[m - 1].yaw;

    /* lift-off flick: keep moving along the end tangent while rising */
    float vx = s_path[m - 1].x - s_path[m - 2].x, vy = s_path[m - 1].y - s_path[m - 2].y, vl = hypotf(vx, vy);
    if (vl > 1e-3f) { vx /= vl; vy /= vl; }
    if (!send("G1 X%.2f Y%.2f Z%.2f F%d", s_path[m - 1].x + 3 * vx, s_path[m - 1].y + 3 * vy,
              b->z_touch + 2.0f, FEED_PAINT)) return false;
    s_paint_dist += total;
    return send("G0 Z%.2f F%d", Z_SAFE_MM, FEED_Z);
}

bool motion_paint_line(float x0, float y0, float x1, float y1, float depth, const brush_t *b) {
    float X0 = CANVAS_X0_MM + x0, Y0 = CANVAS_Y0_MM + y0, X1 = CANVAS_X0_MM + x1, Y1 = CANVAS_Y0_MM + y1;
    s_yaw = nearest_yaw(s_yaw, DEG(atan2f(Y1 - Y0, X1 - X0)));
    return travel(X0 - APPROACH_MM, Y0) &&
           send("G0 %c%.2f F%d", YAW_AXIS, s_yaw, FEED_YAW_DEG) &&
           send("G1 Z%.2f F%d", b->z_touch + 1.0f, FEED_Z) &&
           send("G1 X%.2f Y%.2f Z%.3f F%d", X0, Y0, b->z_touch - depth, FEED_PAINT) &&
           send("G1 X%.2f Y%.2f F%d", X1, Y1, FEED_PAINT) &&
           send("G1 X%.2f Z%.2f F%d", X1 + 3, b->z_touch + 2.0f, FEED_PAINT) &&
           send("G0 Z%.2f F%d", Z_SAFE_MM, FEED_Z);
}

float motion_paint_distance(void) { return s_paint_dist; }
