/*
 * image.h - planar float CIELAB images. Lab is used everywhere because
 * Euclidean distance in Lab ~ perceived color difference (deltaE76), which is
 * what the stroke optimizer minimizes.
 *
 * Memory: one 256x256 Lab image = 768 KiB.
 */
#ifndef LEOGOYA_IMAGE_H
#define LEOGOYA_IMAGE_H

#include <stdint.h>
#include <stdbool.h>

/* One CIELAB color: L = lightness (0..100), a = green..red, b = blue..yellow. */
typedef struct
{
    float L;
    float a;
    float b;
} lab_t;

/* Planar image: one float array per Lab channel, each width*height, row-major. */
typedef struct
{
    int width;
    int height;
    float *L;
    float *a;
    float *b;
} lab_image_t;

bool img_alloc(lab_image_t *image, int width, int height);
void img_free(lab_image_t *image);
void img_copy(lab_image_t *dest, const lab_image_t *source);
void img_fill(lab_image_t *image, lab_t color);

static inline lab_t img_get(const lab_image_t *image, int x, int y)
{
    int index = y * image->width + x;
    lab_t color = { image->L[index], image->a[index], image->b[index] };
    return color;
}

static inline void img_set(lab_image_t *image, int x, int y, lab_t color)
{
    int index = y * image->width + x;
    image->L[index] = color.L;
    image->a[index] = color.a;
    image->b[index] = color.b;
}

/* deltaE76: Euclidean distance between two Lab colors */
float lab_dist(lab_t first, lab_t second);
lab_t rgb_to_lab(uint8_t red, uint8_t green, uint8_t blue);
void lab_to_rgb(lab_t color, uint8_t *red, uint8_t *green, uint8_t *blue);

/* Separable gaussian blur, sigma in px. scratch must hold width*height floats. */
void img_blur(lab_image_t *dest, const lab_image_t *source, float sigma, float *scratch);

/* Sobel gradient of the L (lightness) channel at (x, y) */
void img_gradient_L(const lab_image_t *image, int x, int y, float *grad_x, float *grad_y);

/* Convert to / from 8-bit interleaved RGB (for preview dumps) */
void img_to_rgb8(const lab_image_t *image, uint8_t *rgb);
void img_from_rgb8(lab_image_t *image, const uint8_t *rgb);

#endif
