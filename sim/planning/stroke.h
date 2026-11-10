#ifndef LEOGOYA_STROKE_H
#define LEOGOYA_STROKE_H

#include "config.h"
#include <stdint.h>

/* One brush stroke = a spline through control points, in canvas pixels.
 * The planner decides reloads so the simulator and the machine agree on how
 * much paint is on the brush. */
typedef struct
{
    float x;
    float y;
} stroke_point_t;

typedef enum
{
    RELOAD_NONE = 0,  /* keep painting with what's on the brush */
    RELOAD_DIP = 1,   /* re-dip the same color */
    RELOAD_RINSE = 2, /* rinse, then dip a new color */
} reload_t;

typedef struct
{
    uint8_t color;  /* palette / pot index */
    uint8_t brush;  /* physical brush id */
    uint8_t reload; /* reload_t: what to do before this stroke */
    uint8_t num_points;
    float width_mm; /* body width; motion converts it to press depth */
    stroke_point_t points[STROKE_MAX_PTS];
} stroke_t;

#endif
