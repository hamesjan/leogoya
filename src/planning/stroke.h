#ifndef LEOGOYA_STROKE_H
#define LEOGOYA_STROKE_H

#include "config.h"
#include <stdint.h>

/* One brush stroke = a spline through control points, in canvas pixels.
 * The planner decides reloads so the simulator and the machine agree on how
 * much paint is on the brush. */
typedef struct { float x, y; } spt_t;

typedef struct {
    uint8_t color;        /* palette / pot index */
    uint8_t brush;        /* physical brush id */
    uint8_t reload;       /* 0 = keep going, 1 = re-dip same color, 2 = rinse + new color */
    uint8_t npts;
    float   width_mm;     /* body width; trajectory converts to press depth */
    spt_t   p[STROKE_MAX_PTS];
} stroke_t;

#endif
