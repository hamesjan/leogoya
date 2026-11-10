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

typedef struct
{
    float x;
    float y;
} point2_t;

/* Finds the 4 marker centroids in top-left, top-right, bottom-right, bottom-left order. */
bool vision_find_markers(const cam_frame_t *frame, point2_t corners[4]);

/* 3x3 homography (row-major, last element 1) mapping to_points[i] -> from_points[i]. */
bool vision_homography(const point2_t to_points[4], const point2_t from_points[4], float homography[9]);

/* Detect the markers, warp the quad between them onto rectified (its full
 * width x height), converting to Lab. */
bool vision_rectify(const cam_frame_t *frame, lab_image_t *rectified);

#endif
