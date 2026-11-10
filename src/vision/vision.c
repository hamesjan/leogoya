#include "vision/vision.h"
#include "config.h"
#include <math.h>
#include <string.h>

static inline void rgb565_unpack(uint16_t p, int *r, int *g, int *b) {
    *r = ((p >> 11) & 31) * 255 / 31;
    *g = ((p >> 5) & 63) * 255 / 63;
    *b = (p & 31) * 255 / 31;
}

static inline bool is_marker(int r, int g, int b) {
    int mn = r < b ? r : b;
    return r > 140 && b > 110 && g * 10 < mn * 6;   /* strong magenta */
}

bool vision_find_markers(const cam_frame_t *f, pt2_t c[4]) {
    double sx[4] = {0}, sy[4] = {0};
    int n[4] = {0};
    int hw = f->w / 2, hh = f->h / 2;
    for (int y = 0; y < f->h; y++)
        for (int x = 0; x < f->w; x++) {
            int r, g, b;
            rgb565_unpack(f->rgb565[y * f->w + x], &r, &g, &b);
            if (!is_marker(r, g, b)) continue;
            /* quadrant -> TL0 TR1 BR2 BL3 */
            int q = (y < hh) ? (x < hw ? 0 : 1) : (x < hw ? 3 : 2);
            sx[q] += x; sy[q] += y; n[q]++;
        }
    for (int q = 0; q < 4; q++) {
        if (n[q] < MARKER_MIN_PIXELS) {
            hal_log("vision: marker %d not found (%d px)\n", q, n[q]);
            return false;
        }
        c[q].x = (float)(sx[q] / n[q]);
        c[q].y = (float)(sy[q] / n[q]);
    }
    return true;
}

/* Solve 8x8 A x = b by Gaussian elimination with partial pivoting. */
static bool solve8(double A[8][9]) {
    for (int col = 0; col < 8; col++) {
        int piv = col;
        for (int r = col + 1; r < 8; r++)
            if (fabs(A[r][col]) > fabs(A[piv][col])) piv = r;
        if (fabs(A[piv][col]) < 1e-12) return false;
        if (piv != col)
            for (int k = 0; k < 9; k++) { double t = A[col][k]; A[col][k] = A[piv][k]; A[piv][k] = t; }
        for (int r = 0; r < 8; r++) {
            if (r == col) continue;
            double m = A[r][col] / A[col][col];
            for (int k = col; k < 9; k++) A[r][k] -= m * A[col][k];
        }
    }
    for (int r = 0; r < 8; r++) A[r][8] /= A[r][r];
    return true;
}

bool vision_homography(const pt2_t dst[4], const pt2_t src[4], float h[9]) {
    double A[8][9];
    for (int i = 0; i < 4; i++) {
        double u = dst[i].x, v = dst[i].y, x = src[i].x, y = src[i].y;
        double r0[9] = { u, v, 1, 0, 0, 0, -u * x, -v * x, x };
        double r1[9] = { 0, 0, 0, u, v, 1, -u * y, -v * y, y };
        memcpy(A[2 * i], r0, sizeof r0);
        memcpy(A[2 * i + 1], r1, sizeof r1);
    }
    if (!solve8(A)) return false;
    for (int i = 0; i < 8; i++) h[i] = (float)A[i][8];
    h[8] = 1.0f;
    return true;
}

bool vision_rectify(const cam_frame_t *f, lab_image_t *out) {
    pt2_t src[4];
    if (!vision_find_markers(f, src)) return false;
    pt2_t dst[4] = { {0, 0}, {(float)out->w, 0}, {(float)out->w, (float)out->h}, {0, (float)out->h} };
    float H[9];
    if (!vision_homography(dst, src, H)) return false;

    /* Average a 2x2 grid of camera samples per output pixel (cheap anti-alias). */
    for (int v = 0; v < out->h; v++)
        for (int u = 0; u < out->w; u++) {
            int R = 0, G = 0, B = 0, cnt = 0;
            for (int sy = 0; sy < 2; sy++)
                for (int sx = 0; sx < 2; sx++) {
                    float uu = u + 0.25f + 0.5f * sx, vv = v + 0.25f + 0.5f * sy;
                    float d = H[6] * uu + H[7] * vv + 1.0f;
                    int x = (int)((H[0] * uu + H[1] * vv + H[2]) / d);
                    int y = (int)((H[3] * uu + H[4] * vv + H[5]) / d);
                    if (x < 0 || y < 0 || x >= f->w || y >= f->h) continue;
                    int r, g, b;
                    rgb565_unpack(f->rgb565[y * f->w + x], &r, &g, &b);
                    R += r; G += g; B += b; cnt++;
                }
            if (!cnt) cnt = 1;
            img_set(out, u, v, rgb_to_lab((uint8_t)(R / cnt), (uint8_t)(G / cnt), (uint8_t)(B / cnt)));
        }
    return true;
}
