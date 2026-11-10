/*
 * palette.h - k-means color quantization (the "pick the paints" ML step).
 *
 * The robot cannot mix paint, so the reference is reduced to PALETTE_N colors.
 * The planner prints them as hex + a rough mixing hint; you mix those pots by hand.
 */
#ifndef LEOGOYA_PALETTE_H
#define LEOGOYA_PALETTE_H

#include "vision/image.h"

typedef struct
{
    int count;
    lab_t colors[16];
    float share[16]; /* fraction of reference pixels assigned to each color */
} palette_t;

void palette_kmeans(const lab_image_t *reference, int num_colors, uint32_t seed, palette_t *palette);
int palette_nearest(const palette_t *palette, lab_t color);

void palette_print(const palette_t *palette);

#endif
