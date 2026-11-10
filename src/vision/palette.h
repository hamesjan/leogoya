/*
 * palette.h - k-means color quantization (the "pick the paints" ML step).
 *
 * The robot cannot mix paint, so the reference is reduced to PALETTE_N colors.
 * The firmware prints them as hex + a rough mixing hint; you mix those pots by
 * hand, paint a swatch of each, and palette_measure() re-learns the *actual*
 * colors from a photo so the simulator matches what the brush really lays down.
 */
#ifndef LEOGOYA_PALETTE_H
#define LEOGOYA_PALETTE_H

#include "vision/image.h"

typedef struct {
    int n;
    lab_t c[16];
    float share[16];   /* fraction of reference pixels assigned to each color */
} palette_t;

void palette_kmeans(const lab_image_t *ref, int k, uint32_t seed, palette_t *out);
int  palette_nearest(const palette_t *p, lab_t c);

/* Swatches: a row of PALETTE_N squares painted across the middle of the
 * canvas; samples the center of each. */
void palette_measure(const lab_image_t *canvas, palette_t *p);

void palette_print(const palette_t *p);

#endif
