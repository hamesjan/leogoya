/*
 * image.h - planar float CIELAB images. Lab is used everywhere because
 * Euclidean distance in Lab ~ perceived color difference (deltaE76), which is
 * what the stroke optimizer minimizes.
 *
 * Memory: one 256x256 Lab image = 768 KiB (PSRAM).
 */
#ifndef LEOGOYA_IMAGE_H
#define LEOGOYA_IMAGE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct { float L, a, b; } lab_t;

typedef struct {
    int w, h;
    float *L, *a, *b;   /* each w*h */
} lab_image_t;

bool  img_alloc(lab_image_t *im, int w, int h);
void  img_free(lab_image_t *im);
void  img_copy(lab_image_t *dst, const lab_image_t *src);
void  img_fill(lab_image_t *im, lab_t c);

static inline lab_t img_get(const lab_image_t *im, int x, int y) {
    int i = y * im->w + x;
    lab_t c = { im->L[i], im->a[i], im->b[i] };
    return c;
}
static inline void img_set(lab_image_t *im, int x, int y, lab_t c) {
    int i = y * im->w + x;
    im->L[i] = c.L; im->a[i] = c.a; im->b[i] = c.b;
}
lab_t img_sample_bilinear(const lab_image_t *im, float x, float y);

float lab_dist(lab_t p, lab_t q);
lab_t rgb_to_lab(uint8_t r, uint8_t g, uint8_t b);
void  lab_to_rgb(lab_t c, uint8_t *r, uint8_t *g, uint8_t *b);

/* separable gaussian blur, sigma in px; tmp must be a w*h float scratch buffer */
void  img_blur(lab_image_t *dst, const lab_image_t *src, float sigma, float *tmp);

/* Sobel gradient of L channel at (x,y) */
void  img_gradient_L(const lab_image_t *im, int x, int y, float *gx, float *gy);

/* convert to 8-bit interleaved RGB (for debug dumps / storage) */
void  img_to_rgb8(const lab_image_t *im, uint8_t *rgb);
void  img_from_rgb8(lab_image_t *im, const uint8_t *rgb);

#endif
