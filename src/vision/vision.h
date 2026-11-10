/*
 * vision.h - turn a camera frame into a rectified, top-down Lab image.
 *
 * Put a bright magenta dot (sticker, ~15 mm) on each corner of whatever you
 * photograph: the reference print, or the canvas itself (paint over them last,
 * or put them on the stretcher bars just outside the canvas). The camera must
 * be mounted roughly upright so each marker lands in its own frame quadrant.
 */
#ifndef LEOGOYA_VISION_H
#define LEOGOYA_VISION_H

#include "hal/hal.h"
#include "vision/image.h"
#include <stdbool.h>

typedef struct { float x, y; } pt2_t;

/* Finds the 4 marker centroids in TL, TR, BR, BL order. */
bool vision_find_markers(const cam_frame_t *f, pt2_t corners[4]);

/* Homography mapping dst[i] -> src[i] (3x3, h[8] = 1). */
bool vision_homography(const pt2_t dst[4], const pt2_t src[4], float h[9]);

/* Detect markers, warp the quad to out (out->w x out->h), convert to Lab. */
bool vision_rectify(const cam_frame_t *f, lab_image_t *out);

#endif
