/*
 * hal_host.c - run the whole firmware on a laptop.
 *
 *   ./build/leogoya_sim [reference.ppm] [out_dir]   (either order, both optional)
 *
 * - The "camera" shows the reference (binary PPM, P6) lying on a grey table,
 *   slightly in perspective, with magenta corner dots - so the marker
 *   detection + homography path is exercised for real.
 * - Canvas photos are not available, so closed loop falls back to the simulator.
 * - G-code goes to out_dir/painting.gcode; previews to out_dir as .ppm files.
 * - With no reference argument a synthetic test scene is generated.
 */
#include "hal/hal.h"
#include "vision/vision.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static const char *s_ref_path;
static const char *s_out_dir = "sim_out";
static FILE *s_gcode;
static long s_lines;

static void *s_slot[8];
static size_t s_slot_len[8];
static uint32_t s_boot_counter;

void hal_host_args(int argc, char **argv) {
    /* any *.ppm argument is the reference, anything else the output dir */
    for (int i = 1; i < argc; i++) {
        size_t n = strlen(argv[i]);
        if (n > 4 && !strcmp(argv[i] + n - 4, ".ppm")) s_ref_path = argv[i];
        else if (n) s_out_dir = argv[i];
    }
}

void hal_init(void) {
    char path[512];
    snprintf(path, sizeof path, "mkdir -p '%s'", s_out_dir);
    if (system(path) != 0) { fprintf(stderr, "cannot create %s\n", s_out_dir); exit(1); }
    snprintf(path, sizeof path, "%s/painting.gcode", s_out_dir);
    s_gcode = fopen(path, "w");
    if (!s_gcode) { perror(path); exit(1); }
}

uint32_t hal_millis(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}
void hal_delay_ms(uint32_t ms) { (void)ms; }

void hal_log(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    fflush(stdout);
}

void *hal_alloc_large(size_t n) { return malloc(n); }
void hal_free_large(void *p) { free(p); }

/* ---- camera ---- */
static uint8_t *load_ppm(const char *path, int *w, int *h) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    int maxv;
    if (fscanf(f, "P6 %d %d %d", w, h, &maxv) != 3 || maxv != 255) { fclose(f); return NULL; }
    fgetc(f);
    uint8_t *px = malloc((size_t)*w * *h * 3);
    if (fread(px, 3, (size_t)*w * *h, f) != (size_t)*w * *h) { free(px); px = NULL; }
    fclose(f);
    return px;
}

static uint8_t *synthetic_scene(int *w, int *h) {
    *w = *h = 400;
    uint8_t *px = malloc(400 * 400 * 3);
    for (int y = 0; y < 400; y++)
        for (int x = 0; x < 400; x++) {
            uint8_t *p = &px[(y * 400 + x) * 3];
            float hill = 250 + 40 * __builtin_sinf(x * 0.02f) + 15 * __builtin_sinf(x * 0.07f);
            float sx = x - 280, sy = y - 110;
            if (sx * sx + sy * sy < 45 * 45) { p[0] = 250; p[1] = 200; p[2] = 60; }      /* sun */
            else if (y > hill + 60) { p[0] = 60; p[1] = 90; p[2] = 40; }                  /* field */
            else if (y > hill) { p[0] = 90; p[1] = 140; p[2] = 70; }                       /* hills */
            else { p[0] = (uint8_t)(90 + y / 4); p[1] = (uint8_t)(140 + y / 5); p[2] = 220; } /* sky */
            if (x > 80 && x < 100 && y > hill - 70 && y < hill + 20) { p[0] = 70; p[1] = 45; p[2] = 30; } /* tree */
            float tx = x - 90, ty = y - (hill - 90);
            if (tx * tx + ty * ty < 38 * 38) { p[0] = 40; p[1] = 100; p[2] = 45; }
        }
    return px;
}

bool hal_camera_init(void) { return true; }

bool hal_camera_capture(capture_purpose_t why, cam_frame_t *out) {
    if (why != CAPTURE_REFERENCE) return false;
    int rw, rh;
    uint8_t *ref = s_ref_path ? load_ppm(s_ref_path, &rw, &rh) : synthetic_scene(&rw, &rh);
    if (!ref) { hal_log("host: cannot read %s (must be binary P6 PPM)\n", s_ref_path); return false; }

    const int W = 800, H = 600;
    uint16_t *fb = malloc(sizeof(uint16_t) * W * H);
    pt2_t quad[4] = { {150, 60}, {660, 80}, {700, 560}, {110, 540} };  /* perspective-ish */
    pt2_t rc[4] = { {0, 0}, {(float)rw, 0}, {(float)rw, (float)rh}, {0, (float)rh} };
    float Hm[9];
    vision_homography(quad, rc, Hm);      /* frame px -> reference px */
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            int r = 120, g = 118, b = 112;  /* table */
            float d = Hm[6] * x + Hm[7] * y + 1;
            int u = (int)((Hm[0] * x + Hm[1] * y + Hm[2]) / d), v = (int)((Hm[3] * x + Hm[4] * y + Hm[5]) / d);
            if (u >= 0 && v >= 0 && u < rw && v < rh) {
                uint8_t *p = &ref[(v * rw + u) * 3];
                r = p[0]; g = p[1]; b = p[2];
            }
            for (int k = 0; k < 4; k++) {
                float dx = x - quad[k].x, dy = y - quad[k].y;
                if (dx * dx + dy * dy < 36) { r = 235; g = 20; b = 220; }
            }
            fb[y * W + x] = (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
        }
    free(ref);
    out->w = W; out->h = H; out->rgb565 = fb; out->handle = fb;
    return true;
}

void hal_camera_release(cam_frame_t *f) { free(f->handle); }

/* ---- motion: write G-code to a file ---- */
bool hal_motion_send(const char *line, uint32_t timeout_ms) {
    (void)timeout_ms;
    fprintf(s_gcode, "%s\n", line);
    s_lines++;
    return true;
}

/* ---- operator ---- */
bool hal_button_held(void) { return false; }
void hal_wait_button(const char *prompt) {
    hal_log("[button] %s  (auto-continue in sim)\n", prompt);
    fprintf(s_gcode, "; PAUSE: %s\n", prompt);
}
void hal_led(bool on) { (void)on; }
void hal_halt(void) { fflush(s_gcode); exit(1); }

/* ---- storage: RAM only, every sim run starts fresh ---- */
bool hal_storage_save(int slot, const void *data, size_t len) {
    free(s_slot[slot]);
    s_slot[slot] = malloc(len);
    memcpy(s_slot[slot], data, len);
    s_slot_len[slot] = len;
    return true;
}
bool hal_storage_load(int slot, void *data, size_t len) {
    if (!s_slot[slot] || s_slot_len[slot] != len) return false;
    memcpy(data, s_slot[slot], len);
    return true;
}

reset_reason_t hal_reset_reason(void) { return RESET_POWERON; }
uint32_t hal_boot_counter_get(void) { return s_boot_counter; }
void hal_boot_counter_set(uint32_t v) { s_boot_counter = v; }
void hal_boot_mark_app_valid(void) {}

void hal_debug_dump_rgb(const char *name, int w, int h, const uint8_t *rgb) {
    char path[512];
    snprintf(path, sizeof path, "%s/%s.ppm", s_out_dir, name);
    FILE *f = fopen(path, "wb");
    if (!f) return;
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    fwrite(rgb, 3, (size_t)w * h, f);
    fclose(f);
    hal_log("host: wrote %s  (%ld G-code lines so far)\n", path, s_lines);
}
