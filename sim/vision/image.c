#include "vision/image.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

bool img_alloc(lab_image_t *image, int width, int height)
{
    size_t channel_bytes = (size_t)width * height * sizeof(float);
    image->width = width;
    image->height = height;
    image->L = malloc(channel_bytes);
    image->a = malloc(channel_bytes);
    image->b = malloc(channel_bytes);
    if (!image->L || !image->a || !image->b)
    {
        img_free(image);
        return false;
    }
    return true;
}

void img_free(lab_image_t *image)
{
    free(image->L);
    free(image->a);
    free(image->b);
    image->L = 0;
    image->a = 0;
    image->b = 0;
}

void img_copy(lab_image_t *dest, const lab_image_t *source)
{
    size_t channel_bytes = (size_t)source->width * source->height * sizeof(float);
    memcpy(dest->L, source->L, channel_bytes);
    memcpy(dest->a, source->a, channel_bytes);
    memcpy(dest->b, source->b, channel_bytes);
}

void img_fill(lab_image_t *image, lab_t color)
{
    int pixel_count = image->width * image->height;
    for (int i = 0; i < pixel_count; i++)
    {
        image->L[i] = color.L;
        image->a[i] = color.a;
        image->b[i] = color.b;
    }
}

float lab_dist(lab_t first, lab_t second)
{
    float delta_L = first.L - second.L;
    float delta_a = first.a - second.a;
    float delta_b = first.b - second.b;
    return sqrtf(delta_L * delta_L + delta_a * delta_a + delta_b * delta_b);
}

/* ---- sRGB <-> CIELAB (D65 white point) ---- */

static float srgb_to_linear(float channel)
{
    return channel <= 0.04045f ? channel / 12.92f : powf((channel + 0.055f) / 1.055f, 2.4f);
}

static float linear_to_srgb(float channel)
{
    return channel <= 0.0031308f ? 12.92f * channel : 1.055f * powf(channel, 1.0f / 2.4f) - 0.055f;
}

/* CIE f(t) used by the XYZ -> Lab conversion, and its inverse */
static float lab_f(float t)
{
    return t > 0.008856f ? cbrtf(t) : (7.787f * t + 16.0f / 116.0f);
}

static float lab_f_inverse(float t)
{
    float t_cubed = t * t * t;
    return t_cubed > 0.008856f ? t_cubed : (t - 16.0f / 116.0f) / 7.787f;
}

/* 8-bit sRGB -> linear, cached: rgb_to_lab runs on every camera pixel */
static float s_srgb_to_linear_lut[256];
static bool s_lut_ready;

lab_t rgb_to_lab(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!s_lut_ready)
    {
        for (int i = 0; i < 256; i++)
        {
            s_srgb_to_linear_lut[i] = srgb_to_linear(i / 255.0f);
        }
        s_lut_ready = true;
    }
    float r = s_srgb_to_linear_lut[red];
    float g = s_srgb_to_linear_lut[green];
    float b = s_srgb_to_linear_lut[blue];
    /* linear RGB -> CIE XYZ, normalized to the D65 white point */
    float X = (0.4124f * r + 0.3576f * g + 0.1805f * b) / 0.95047f;
    float Y = (0.2126f * r + 0.7152f * g + 0.0722f * b);
    float Z = (0.0193f * r + 0.1192f * g + 0.9505f * b) / 1.08883f;
    float fx = lab_f(X);
    float fy = lab_f(Y);
    float fz = lab_f(Z);
    lab_t color = { 116.0f * fy - 16.0f, 500.0f * (fx - fy), 200.0f * (fy - fz) };
    return color;
}

/* 0..1 float -> 0..255 byte, rounded and clamped */
static uint8_t unit_to_byte(float value)
{
    value = value * 255.0f + 0.5f;
    if (value < 0)
    {
        return 0;
    }
    if (value > 255)
    {
        return 255;
    }
    return (uint8_t)value;
}

void lab_to_rgb(lab_t color, uint8_t *red, uint8_t *green, uint8_t *blue)
{
    float fy = (color.L + 16.0f) / 116.0f;
    float fx = fy + color.a / 500.0f;
    float fz = fy - color.b / 200.0f;
    float X = lab_f_inverse(fx) * 0.95047f;
    float Y = lab_f_inverse(fy);
    float Z = lab_f_inverse(fz) * 1.08883f;
    /* CIE XYZ -> linear RGB */
    float r = 3.2406f * X - 1.5372f * Y - 0.4986f * Z;
    float g = -0.9689f * X + 1.8758f * Y + 0.0415f * Z;
    float b = 0.0557f * X - 0.2040f * Y + 1.0570f * Z;
    *red = unit_to_byte(linear_to_srgb(r < 0 ? 0 : r));
    *green = unit_to_byte(linear_to_srgb(g < 0 ? 0 : g));
    *blue = unit_to_byte(linear_to_srgb(b < 0 ? 0 : b));
}

/* ---- blur ---- */

static int clamp_index(int index, int size)
{
    if (index < 0)
    {
        return 0;
    }
    if (index >= size)
    {
        return size - 1;
    }
    return index;
}

static void blur_channel(float *dest, const float *source, int width, int height, float sigma, float *scratch)
{
    if (sigma < 0.3f)
    {
        memcpy(dest, source, (size_t)width * height * sizeof(float));
        return;
    }
    int radius = (int)ceilf(sigma * 2.5f);
    if (radius > 31)
    {
        radius = 31;
    }
    float kernel[64];
    float kernel_sum = 0;
    for (int offset = -radius; offset <= radius; offset++)
    {
        kernel[offset + radius] = expf(-(offset * offset) / (2 * sigma * sigma));
        kernel_sum += kernel[offset + radius];
    }
    for (int i = 0; i <= 2 * radius; i++)
    {
        kernel[i] /= kernel_sum;
    }
    /* horizontal pass: source -> scratch */
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            float sum = 0;
            for (int offset = -radius; offset <= radius; offset++)
            {
                sum += kernel[offset + radius] * source[y * width + clamp_index(x + offset, width)];
            }
            scratch[y * width + x] = sum;
        }
    }
    /* vertical pass: scratch -> dest */
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            float sum = 0;
            for (int offset = -radius; offset <= radius; offset++)
            {
                sum += kernel[offset + radius] * scratch[clamp_index(y + offset, height) * width + x];
            }
            dest[y * width + x] = sum;
        }
    }
}

void img_blur(lab_image_t *dest, const lab_image_t *source, float sigma, float *scratch)
{
    blur_channel(dest->L, source->L, source->width, source->height, sigma, scratch);
    blur_channel(dest->a, source->a, source->width, source->height, sigma, scratch);
    blur_channel(dest->b, source->b, source->width, source->height, sigma, scratch);
}

/* Lightness at (x, y), with coordinates clamped to the image edge */
static float lightness_at(const lab_image_t *image, int x, int y)
{
    return image->L[clamp_index(y, image->height) * image->width + clamp_index(x, image->width)];
}

void img_gradient_L(const lab_image_t *image, int x, int y, float *grad_x, float *grad_y)
{
    /* Sobel operator */
    *grad_x = (lightness_at(image, x + 1, y - 1) + 2 * lightness_at(image, x + 1, y) +
               lightness_at(image, x + 1, y + 1)) -
              (lightness_at(image, x - 1, y - 1) + 2 * lightness_at(image, x - 1, y) +
               lightness_at(image, x - 1, y + 1));
    *grad_y = (lightness_at(image, x - 1, y + 1) + 2 * lightness_at(image, x, y + 1) +
               lightness_at(image, x + 1, y + 1)) -
              (lightness_at(image, x - 1, y - 1) + 2 * lightness_at(image, x, y - 1) +
               lightness_at(image, x + 1, y - 1));
}

void img_to_rgb8(const lab_image_t *image, uint8_t *rgb)
{
    int pixel_count = image->width * image->height;
    for (int i = 0; i < pixel_count; i++)
    {
        lab_t color = { image->L[i], image->a[i], image->b[i] };
        lab_to_rgb(color, &rgb[3 * i], &rgb[3 * i + 1], &rgb[3 * i + 2]);
    }
}

void img_from_rgb8(lab_image_t *image, const uint8_t *rgb)
{
    int pixel_count = image->width * image->height;
    for (int i = 0; i < pixel_count; i++)
    {
        lab_t color = rgb_to_lab(rgb[3 * i], rgb[3 * i + 1], rgb[3 * i + 2]);
        image->L[i] = color.L;
        image->a[i] = color.a;
        image->b[i] = color.b;
    }
}
