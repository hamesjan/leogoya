#include "vision/image.h"
#include "hal/hal.h"
#include <math.h>
#include <string.h>

bool img_alloc(lab_image_t *im, int w, int h) {
    size_t n = (size_t)w * h * sizeof(float);
    im->w = w; im->h = h;
    im->L = hal_alloc_large(n);
    im->a = hal_alloc_large(n);
    im->b = hal_alloc_large(n);
    if (!im->L || !im->a || !im->b) { img_free(im); return false; }
    return true;
}

void img_free(lab_image_t *im) {
    hal_free_large(im->L); hal_free_large(im->a); hal_free_large(im->b);
    im->L = im->a = im->b = 0;
}

void img_copy(lab_image_t *dst, const lab_image_t *src) {
    size_t n = (size_t)src->w * src->h * sizeof(float);
    memcpy(dst->L, src->L, n); memcpy(dst->a, src->a, n); memcpy(dst->b, src->b, n);
}

void img_fill(lab_image_t *im, lab_t c) {
    int n = im->w * im->h;
    for (int i = 0; i < n; i++) { im->L[i] = c.L; im->a[i] = c.a; im->b[i] = c.b; }
}

lab_t img_sample_bilinear(const lab_image_t *im, float x, float y) {
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x > im->w - 1.001f) x = im->w - 1.001f;
    if (y > im->h - 1.001f) y = im->h - 1.001f;
    int x0 = (int)x, y0 = (int)y;
    float fx = x - x0, fy = y - y0;
    int i00 = y0 * im->w + x0, i10 = i00 + 1, i01 = i00 + im->w, i11 = i01 + 1;
    float w00 = (1 - fx) * (1 - fy), w10 = fx * (1 - fy), w01 = (1 - fx) * fy, w11 = fx * fy;
    lab_t c;
    c.L = im->L[i00] * w00 + im->L[i10] * w10 + im->L[i01] * w01 + im->L[i11] * w11;
    c.a = im->a[i00] * w00 + im->a[i10] * w10 + im->a[i01] * w01 + im->a[i11] * w11;
    c.b = im->b[i00] * w00 + im->b[i10] * w10 + im->b[i01] * w01 + im->b[i11] * w11;
    return c;
}

float lab_dist(lab_t p, lab_t q) {
    float dL = p.L - q.L, da = p.a - q.a, db = p.b - q.b;
    return sqrtf(dL * dL + da * da + db * db);
}

/* ---- sRGB <-> CIELAB (D65) ---- */
static float srgb_to_lin(float c) {
    return c <= 0.04045f ? c / 12.92f : powf((c + 0.055f) / 1.055f, 2.4f);
}
static float lin_to_srgb(float c) {
    return c <= 0.0031308f ? 12.92f * c : 1.055f * powf(c, 1.0f / 2.4f) - 0.055f;
}
static float f_lab(float t) {
    return t > 0.008856f ? cbrtf(t) : (7.787f * t + 16.0f / 116.0f);
}
static float f_lab_inv(float t) {
    float t3 = t * t * t;
    return t3 > 0.008856f ? t3 : (t - 16.0f / 116.0f) / 7.787f;
}

/* 8-bit sRGB -> linear, cached: rgb_to_lab runs on every camera pixel */
static float s_lin_lut[256];
static bool s_lut_ready;

lab_t rgb_to_lab(uint8_t r8, uint8_t g8, uint8_t b8) {
    if (!s_lut_ready) {
        for (int i = 0; i < 256; i++) s_lin_lut[i] = srgb_to_lin(i / 255.0f);
        s_lut_ready = true;
    }
    float r = s_lin_lut[r8], g = s_lin_lut[g8], b = s_lin_lut[b8];
    float X = (0.4124f * r + 0.3576f * g + 0.1805f * b) / 0.95047f;
    float Y = (0.2126f * r + 0.7152f * g + 0.0722f * b);
    float Z = (0.0193f * r + 0.1192f * g + 0.9505f * b) / 1.08883f;
    float fx = f_lab(X), fy = f_lab(Y), fz = f_lab(Z);
    lab_t c = { 116.0f * fy - 16.0f, 500.0f * (fx - fy), 200.0f * (fy - fz) };
    return c;
}

static uint8_t clamp8(float v) {
    v = v * 255.0f + 0.5f;
    return v < 0 ? 0 : v > 255 ? 255 : (uint8_t)v;
}

void lab_to_rgb(lab_t c, uint8_t *r8, uint8_t *g8, uint8_t *b8) {
    float fy = (c.L + 16.0f) / 116.0f, fx = fy + c.a / 500.0f, fz = fy - c.b / 200.0f;
    float X = f_lab_inv(fx) * 0.95047f, Y = f_lab_inv(fy), Z = f_lab_inv(fz) * 1.08883f;
    float r =  3.2406f * X - 1.5372f * Y - 0.4986f * Z;
    float g = -0.9689f * X + 1.8758f * Y + 0.0415f * Z;
    float b =  0.0557f * X - 0.2040f * Y + 1.0570f * Z;
    *r8 = clamp8(lin_to_srgb(r < 0 ? 0 : r));
    *g8 = clamp8(lin_to_srgb(g < 0 ? 0 : g));
    *b8 = clamp8(lin_to_srgb(b < 0 ? 0 : b));
}

/* ---- blur ---- */
static void blur_channel(float *dst, const float *src, int w, int h, float sigma, float *tmp) {
    if (sigma < 0.3f) { memcpy(dst, src, (size_t)w * h * sizeof(float)); return; }
    int r = (int)ceilf(sigma * 2.5f);
    if (r > 31) r = 31;
    float k[64], ks = 0;
    for (int i = -r; i <= r; i++) { k[i + r] = expf(-(i * i) / (2 * sigma * sigma)); ks += k[i + r]; }
    for (int i = 0; i <= 2 * r; i++) k[i] /= ks;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            float s = 0;
            for (int i = -r; i <= r; i++) {
                int xx = x + i; xx = xx < 0 ? 0 : xx >= w ? w - 1 : xx;
                s += k[i + r] * src[y * w + xx];
            }
            tmp[y * w + x] = s;
        }
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            float s = 0;
            for (int i = -r; i <= r; i++) {
                int yy = y + i; yy = yy < 0 ? 0 : yy >= h ? h - 1 : yy;
                s += k[i + r] * tmp[yy * w + x];
            }
            dst[y * w + x] = s;
        }
}

void img_blur(lab_image_t *dst, const lab_image_t *src, float sigma, float *tmp) {
    blur_channel(dst->L, src->L, src->w, src->h, sigma, tmp);
    blur_channel(dst->a, src->a, src->w, src->h, sigma, tmp);
    blur_channel(dst->b, src->b, src->w, src->h, sigma, tmp);
}

void img_gradient_L(const lab_image_t *im, int x, int y, float *gx, float *gy) {
    int w = im->w, h = im->h;
#define P(xx, yy) im->L[((yy) < 0 ? 0 : (yy) >= h ? h - 1 : (yy)) * w + \
                        ((xx) < 0 ? 0 : (xx) >= w ? w - 1 : (xx))]
    *gx = (P(x + 1, y - 1) + 2 * P(x + 1, y) + P(x + 1, y + 1))
        - (P(x - 1, y - 1) + 2 * P(x - 1, y) + P(x - 1, y + 1));
    *gy = (P(x - 1, y + 1) + 2 * P(x, y + 1) + P(x + 1, y + 1))
        - (P(x - 1, y - 1) + 2 * P(x, y - 1) + P(x + 1, y - 1));
#undef P
}

void img_to_rgb8(const lab_image_t *im, uint8_t *rgb) {
    int n = im->w * im->h;
    for (int i = 0; i < n; i++) {
        lab_t c = { im->L[i], im->a[i], im->b[i] };
        lab_to_rgb(c, &rgb[3 * i], &rgb[3 * i + 1], &rgb[3 * i + 2]);
    }
}

void img_from_rgb8(lab_image_t *im, const uint8_t *rgb) {
    int n = im->w * im->h;
    for (int i = 0; i < n; i++) {
        lab_t c = rgb_to_lab(rgb[3 * i], rgb[3 * i + 1], rgb[3 * i + 2]);
        im->L[i] = c.L; im->a[i] = c.a; im->b[i] = c.b;
    }
}
