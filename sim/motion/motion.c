#include "motion/motion.h"
#include "config.h"
#include "hal/hal.h"
#include "common/util.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#define RAD_TO_DEG(radians) ((radians) * 57.29577951f)
#define TIMEOUT_MOVE_MS     30000u
#define TIMEOUT_HOME_MS     120000u
#define MAX_PATH_POINTS     512

static float s_yaw_deg = 0;           /* current yaw command, unwrapped */
static float s_paint_distance_mm = 0; /* total stroke path painted (stats) */

/* Format one G-code line and send it; logs on failure. */
static bool send(const char *format, ...)
{
    char line[96];
    va_list args;
    va_start(args, format);
    vsnprintf(line, sizeof line, format, args);
    va_end(args);
    bool ok = hal_motion_send(line, TIMEOUT_MOVE_MS);
    if (!ok)
    {
        hal_log("motion: '%s' failed\n", line);
    }
    return ok;
}

/* canvas pixels -> machine millimetres */
static inline float machine_x(float canvas_px_x)
{
    return CANVAS_X0_MM + canvas_px_x * MM_PER_PX;
}

static inline float machine_y(float canvas_px_y)
{
    return CANVAS_Y0_MM + canvas_px_y * MM_PER_PX;
}

/* A flat brush is symmetric under 180 deg rotation, so pick the equivalent
 * angle nearest the current yaw: minimizes rotation and cable wind-up. */
static float nearest_yaw(float current_deg, float tangent_deg)
{
#if BRUSH_USES_YAW
    float yaw = tangent_deg;
    while (yaw - current_deg > 90.0f)
    {
        yaw -= 180.0f;
    }
    while (yaw - current_deg < -90.0f)
    {
        yaw += 180.0f;
    }
    return yaw;
#else
    (void)tangent_deg;
    return current_deg;
#endif
}

bool motion_init(void)
{
    bool ok = send("G21") && send("G90");
    ok = ok && hal_motion_send("G28", TIMEOUT_HOME_MS);
    ok = ok && send("G0 Z%.2f F%d", Z_SAFE_MM, FEED_Z);
    ok = ok && send("G92 %c0", YAW_AXIS); /* yaw homed by hand: brush flat edge along Y */
    s_yaw_deg = 0;
    return ok;
}

/* Lift to the safe height, then move to (x, y) in machine mm. */
static bool travel(float x, float y)
{
    return send("G0 Z%.2f F%d", Z_SAFE_MM, FEED_Z) && send("G0 X%.2f Y%.2f F%d", x, y, FEED_TRAVEL);
}

bool motion_park(void)
{
    /* unwind yaw while parked so it never accumulates past a turn */
    s_yaw_deg = fmodf(s_yaw_deg, 180.0f);
    /* "ok" means a move was accepted, not finished: M400 blocks until
     * the gantry has actually stopped, so the camera never sees it mid-move. */
    return travel(PARK_X_MM, PARK_Y_MM) && send("G0 %c%.2f F%d", YAW_AXIS, s_yaw_deg, FEED_YAW_DEG) &&
           hal_motion_send("M400", TIMEOUT_HOME_MS);
}

/* Lower to dip_z and circle (8 points per turn) around (center_x, center_y). */
static bool swirl(float center_x, float center_y, float dip_z, float radius, int turns)
{
    if (!send("G1 Z%.2f F%d", dip_z, FEED_Z))
    {
        return false;
    }
    for (int point = 0; point < turns * 8; point++)
    {
        float angle = point * 0.785398f; /* 45 degrees */
        if (!send("G1 X%.2f Y%.2f F%d", center_x + radius * cosf(angle), center_y + radius * sinf(angle),
                  FEED_PAINT))
        {
            return false;
        }
    }
    return send("G0 Z%.2f F%d", Z_SAFE_MM, FEED_Z);
}

bool motion_rinse(void)
{
    /* water: swirl; sponge: two drags to wick water out of the bristles */
    if (!travel(WATER_X_MM, WATER_Y_MM) || !swirl(WATER_X_MM, WATER_Y_MM, WATER_DIP_Z_MM, 10, 4))
    {
        return false;
    }
    for (int drag = 0; drag < 2; drag++)
    {
        if (!travel(SPONGE_X_MM - 30, SPONGE_Y_MM + drag * 12) ||
            !send("G1 Z%.2f F%d", SPONGE_Z_MM, FEED_Z) || !send("G1 X%.2f F%d", SPONGE_X_MM + 30, FEED_PAINT))
        {
            return false;
        }
    }
    return send("G0 Z%.2f F%d", Z_SAFE_MM, FEED_Z);
}

bool motion_load_paint(int pot, bool rinse_first)
{
    if (rinse_first && !motion_rinse())
    {
        return false;
    }
    float pot_x = POT_X_MM;
    float pot_y = POT_Y0_MM + pot * POT_PITCH_MM;
    if (!travel(pot_x, pot_y) || !swirl(pot_x, pot_y, POT_DIP_Z_MM, 6, 2))
    {
        return false;
    }
    /* scrape excess on the pot rim: lower to rim height, drag outward */
    return send("G1 Z%.2f F%d", POT_DIP_Z_MM + 8.0f, FEED_Z) &&
           send("G1 X%.2f F%d", pot_x + POT_RADIUS_MM + 4.0f, FEED_PAINT / 2) &&
           send("G0 Z%.2f F%d", Z_SAFE_MM, FEED_Z);
}

/* ---- stroke trajectory ---- */

typedef struct
{
    float x;          /* machine mm */
    float y;          /* machine mm */
    float arc_length; /* mm from the start of the path */
    float yaw_deg;
} path_point_t;

static path_point_t s_path[MAX_PATH_POINTS];

/* Uniform Catmull-Rom spline through the stroke's control points, resampled
 * every PATH_STEP_MM. Returns the number of path points written to s_path. */
static int build_path(const stroke_t *stroke)
{
    int num_controls = stroke->num_points;
    int num_path = 0;
    if (num_controls == 1)
    {
        /* a dab: make a tiny straight stroke */
        float x = machine_x(stroke->points[0].x);
        float y = machine_y(stroke->points[0].y);
        s_path[num_path++] = (path_point_t){ x - 1, y, 0, 0 };
        s_path[num_path++] = (path_point_t){ x + 1, y, 2, 0 };
        return num_path;
    }
    for (int i = 0; i < num_controls - 1 && num_path < MAX_PATH_POINTS - 1; i++)
    {
        /* segment p1 -> p2; p0 and p3 are its neighbours (clamped at the ends) */
        stroke_point_t p0 = stroke->points[i > 0 ? i - 1 : 0];
        stroke_point_t p1 = stroke->points[i];
        stroke_point_t p2 = stroke->points[i + 1];
        stroke_point_t p3 = stroke->points[i + 2 < num_controls ? i + 2 : num_controls - 1];
        float segment_mm = hypotf(p2.x - p1.x, p2.y - p1.y) * MM_PER_PX;
        int samples = (int)ceilf(segment_mm / PATH_STEP_MM);
        if (samples < 1)
        {
            samples = 1;
        }
        for (int sample = (i == 0 ? 0 : 1); sample <= samples && num_path < MAX_PATH_POINTS; sample++)
        {
            float t = (float)sample / samples;
            float t2 = t * t;
            float t3 = t2 * t;
            float x = 0.5f * (2 * p1.x + (-p0.x + p2.x) * t + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 +
                              (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3);
            float y = 0.5f * (2 * p1.y + (-p0.y + p2.y) * t + (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 +
                              (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3);
            s_path[num_path++] = (path_point_t){ machine_x(x), machine_y(y), 0, 0 };
        }
    }
    /* arc length + tangent yaw (unwrapped within the stroke, 180-deg symmetric) */
    float yaw = s_yaw_deg;
    for (int i = 0; i < num_path; i++)
    {
        int before = i > 0 ? i - 1 : 0;
        int after = i < num_path - 1 ? i + 1 : num_path - 1;
        if (i > 0)
        {
            s_path[i].arc_length = s_path[i - 1].arc_length +
                                   hypotf(s_path[i].x - s_path[i - 1].x, s_path[i].y - s_path[i - 1].y);
        }
        float tangent_deg =
            RAD_TO_DEG(atan2f(s_path[after].y - s_path[before].y, s_path[after].x - s_path[before].x));
        yaw = nearest_yaw(yaw, tangent_deg);
        if (i == 0)
        {
            /* re-center between strokes so yaw never winds up */
            while (yaw > 180.0f)
            {
                yaw -= 180.0f;
            }
            while (yaw < -180.0f)
            {
                yaw += 180.0f;
            }
        }
        s_path[i].yaw_deg = yaw;
    }
    return num_path;
}

/* Press profile: ramp in over LANDING_MM, hold, taper out over LIFTOFF_MM. */
static float depth_at(float arc_length, float total_length, float body_depth)
{
    float landing = clampf(arc_length / LANDING_MM, 0, 1);
    float liftoff = clampf((total_length - arc_length) / LIFTOFF_MM, 0, 1);
    float pressure = landing < liftoff ? landing : liftoff;
    return body_depth * (0.35f + 0.65f * pressure); /* never fully lose contact mid-stroke */
}

bool motion_paint_stroke(const stroke_t *stroke, const brush_t *brush)
{
    int num_path = build_path(stroke);
    if (num_path < 2)
    {
        return true;
    }
    const path_point_t *first = &s_path[0];
    const path_point_t *last = &s_path[num_path - 1];
    float total_length = last->arc_length;
    float body_depth = brush_depth_for_width(brush, stroke->width_mm);

    /* approach from behind the start along the initial tangent, brush hovering */
    float start_dir_x = s_path[1].x - first->x;
    float start_dir_y = s_path[1].y - first->y;
    float start_dir_length = hypotf(start_dir_x, start_dir_y);
    if (start_dir_length < 1e-3f)
    {
        start_dir_x = 1;
        start_dir_y = 0;
        start_dir_length = 1;
    }
    start_dir_x /= start_dir_length;
    start_dir_y /= start_dir_length;
    if (!travel(first->x - APPROACH_MM * start_dir_x, first->y - APPROACH_MM * start_dir_y))
    {
        return false;
    }
    if (!send("G0 %c%.2f F%d", YAW_AXIS, first->yaw_deg, FEED_YAW_DEG))
    {
        return false;
    }
    if (!send("G1 Z%.2f F%d", brush->z_touch + 1.0f, FEED_Z))
    {
        return false;
    }

    /* touch down while already moving: avoids the blob a vertical plunge leaves */
    for (int i = 0; i < num_path; i++)
    {
        float z = brush->z_touch - depth_at(s_path[i].arc_length, total_length, body_depth);
        if (!send("G1 X%.2f Y%.2f Z%.3f %c%.2f F%d", s_path[i].x, s_path[i].y, z, YAW_AXIS, s_path[i].yaw_deg,
                  FEED_PAINT))
        {
            return false;
        }
    }
    s_yaw_deg = last->yaw_deg;

    /* lift-off flick: keep moving along the end tangent while rising */
    float end_dir_x = last->x - s_path[num_path - 2].x;
    float end_dir_y = last->y - s_path[num_path - 2].y;
    float end_dir_length = hypotf(end_dir_x, end_dir_y);
    if (end_dir_length > 1e-3f)
    {
        end_dir_x /= end_dir_length;
        end_dir_y /= end_dir_length;
    }
    if (!send("G1 X%.2f Y%.2f Z%.2f F%d", last->x + 3 * end_dir_x, last->y + 3 * end_dir_y,
              brush->z_touch + 2.0f, FEED_PAINT))
    {
        return false;
    }
    s_paint_distance_mm += total_length;
    return send("G0 Z%.2f F%d", Z_SAFE_MM, FEED_Z);
}

bool motion_paint_line(float start_x, float start_y, float end_x, float end_y, float depth_mm,
                       const brush_t *brush)
{
    float from_x = CANVAS_X0_MM + start_x;
    float from_y = CANVAS_Y0_MM + start_y;
    float to_x = CANVAS_X0_MM + end_x;
    float to_y = CANVAS_Y0_MM + end_y;
    s_yaw_deg = nearest_yaw(s_yaw_deg, RAD_TO_DEG(atan2f(to_y - from_y, to_x - from_x)));
    return travel(from_x - APPROACH_MM, from_y) && send("G0 %c%.2f F%d", YAW_AXIS, s_yaw_deg, FEED_YAW_DEG) &&
           send("G1 Z%.2f F%d", brush->z_touch + 1.0f, FEED_Z) &&
           send("G1 X%.2f Y%.2f Z%.3f F%d", from_x, from_y, brush->z_touch - depth_mm, FEED_PAINT) &&
           send("G1 X%.2f Y%.2f F%d", to_x, to_y, FEED_PAINT) &&
           send("G1 X%.2f Z%.2f F%d", to_x + 3, brush->z_touch + 2.0f, FEED_PAINT) &&
           send("G0 Z%.2f F%d", Z_SAFE_MM, FEED_Z);
}

float motion_paint_distance(void)
{
    return s_paint_distance_mm;
}
