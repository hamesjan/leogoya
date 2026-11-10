/*
 * hal_host.c - run the simulator on a laptop.
 *
 *   ./build/sim/leogoya_sim [reference.ppm] [out_dir] [--calibrate]
 *   (any order, all optional)
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

static const char *s_reference_path; /* NULL = use the synthetic scene */
static const char *s_out_dir = "build/sim/out";
static FILE *s_gcode_file;
static long s_gcode_lines;
static bool s_calibrate;

void hal_host_args(int argc, char **argv)
{
    /* any *.ppm argument is the reference, anything else the output dir */
    for (int i = 1; i < argc; i++)
    {
        const char *arg = argv[i];
        size_t length = strlen(arg);
        if (!strcmp(arg, "--calibrate"))
        {
            s_calibrate = true;
        }
        else if (length > 4 && !strcmp(arg + length - 4, ".ppm"))
        {
            s_reference_path = arg;
        }
        else if (length)
        {
            s_out_dir = arg;
        }
    }
}

void hal_init(void)
{
    char path[512];
    snprintf(path, sizeof path, "mkdir -p '%s'", s_out_dir);
    if (system(path) != 0)
    {
        fprintf(stderr, "cannot create %s\n", s_out_dir);
        exit(1);
    }
    snprintf(path, sizeof path, "%s/painting.gcode", s_out_dir);
    s_gcode_file = fopen(path, "w");
    if (!s_gcode_file)
    {
        perror(path);
        exit(1);
    }
}

uint32_t hal_millis(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint32_t)(now.tv_sec * 1000 + now.tv_nsec / 1000000);
}

void hal_log(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    fflush(stdout);
}

/* ---- camera ---- */

/* Load a binary (P6) PPM as interleaved 8-bit RGB. Caller frees. */
static uint8_t *load_ppm(const char *path, int *width, int *height)
{
    FILE *file = fopen(path, "rb");
    if (!file)
    {
        return NULL;
    }
    int max_value;
    if (fscanf(file, "P6 %d %d %d", width, height, &max_value) != 3 || max_value != 255)
    {
        fclose(file);
        return NULL;
    }
    fgetc(file); /* single whitespace byte before the pixel data */
    size_t pixel_count = (size_t)*width * *height;
    uint8_t *pixels = malloc(pixel_count * 3);
    if (fread(pixels, 3, pixel_count, file) != pixel_count)
    {
        free(pixels);
        pixels = NULL;
    }
    fclose(file);
    return pixels;
}

static void set_rgb(uint8_t *pixel, uint8_t red, uint8_t green, uint8_t blue)
{
    pixel[0] = red;
    pixel[1] = green;
    pixel[2] = blue;
}

/* A 400x400 landscape (sky, sun, hills, field, tree) for testing without a reference file. */
static uint8_t *synthetic_scene(int *width, int *height)
{
    const int size = 400;
    *width = size;
    *height = size;
    uint8_t *pixels = malloc((size_t)size * size * 3);
    for (int y = 0; y < size; y++)
    {
        for (int x = 0; x < size; x++)
        {
            uint8_t *pixel = &pixels[(y * size + x) * 3];
            float hill_top = 250 + 40 * __builtin_sinf(x * 0.02f) + 15 * __builtin_sinf(x * 0.07f);
            float from_sun_x = x - 280;
            float from_sun_y = y - 110;
            if (from_sun_x * from_sun_x + from_sun_y * from_sun_y < 45 * 45)
            {
                set_rgb(pixel, 250, 200, 60); /* sun */
            }
            else if (y > hill_top + 60)
            {
                set_rgb(pixel, 60, 90, 40); /* field */
            }
            else if (y > hill_top)
            {
                set_rgb(pixel, 90, 140, 70); /* hills */
            }
            else
            {
                set_rgb(pixel, (uint8_t)(90 + y / 4), (uint8_t)(140 + y / 5), 220); /* sky gradient */
            }
            if (x > 80 && x < 100 && y > hill_top - 70 && y < hill_top + 20)
            {
                set_rgb(pixel, 70, 45, 30); /* tree trunk */
            }
            float from_crown_x = x - 90;
            float from_crown_y = y - (hill_top - 90);
            if (from_crown_x * from_crown_x + from_crown_y * from_crown_y < 38 * 38)
            {
                set_rgb(pixel, 40, 100, 45); /* tree crown */
            }
        }
    }
    return pixels;
}

bool hal_camera_init(void)
{
    return true;
}

/* Fake camera: the reference lying on a grey table in slight perspective,
 * with magenta dots on its corners, as an 800x600 RGB565 frame. */
bool hal_camera_capture(capture_purpose_t purpose, cam_frame_t *frame)
{
    if (purpose != CAPTURE_REFERENCE)
    {
        return false; /* no real canvas to photograph */
    }
    int ref_width;
    int ref_height;
    uint8_t *reference = s_reference_path ? load_ppm(s_reference_path, &ref_width, &ref_height)
                                          : synthetic_scene(&ref_width, &ref_height);
    if (!reference)
    {
        hal_log("host: cannot read %s (must be binary P6 PPM)\n", s_reference_path);
        return false;
    }

    const int frame_width = 800;
    const int frame_height = 600;
    uint16_t *pixels = malloc(sizeof(uint16_t) * frame_width * frame_height);
    point2_t print_corners[4] = { { 150, 60 }, { 660, 80 }, { 700, 560 }, { 110, 540 } }; /* in the frame */
    point2_t ref_corners[4] = {
        { 0, 0 }, { (float)ref_width, 0 }, { (float)ref_width, (float)ref_height }, { 0, (float)ref_height }
    };
    float frame_to_ref[9];
    vision_homography(print_corners, ref_corners, frame_to_ref);
    const float *h = frame_to_ref;
    for (int y = 0; y < frame_height; y++)
    {
        for (int x = 0; x < frame_width; x++)
        {
            int red = 120; /* grey table */
            int green = 118;
            int blue = 112;
            float denominator = h[6] * x + h[7] * y + 1;
            int ref_x = (int)((h[0] * x + h[1] * y + h[2]) / denominator);
            int ref_y = (int)((h[3] * x + h[4] * y + h[5]) / denominator);
            if (ref_x >= 0 && ref_y >= 0 && ref_x < ref_width && ref_y < ref_height)
            {
                uint8_t *ref_pixel = &reference[(ref_y * ref_width + ref_x) * 3];
                red = ref_pixel[0];
                green = ref_pixel[1];
                blue = ref_pixel[2];
            }
            for (int corner = 0; corner < 4; corner++)
            {
                float dx = x - print_corners[corner].x;
                float dy = y - print_corners[corner].y;
                if (dx * dx + dy * dy < 36)
                {
                    red = 235; /* magenta marker dot, radius 6 px */
                    green = 20;
                    blue = 220;
                }
            }
            pixels[y * frame_width + x] = (uint16_t)(((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3));
        }
    }
    free(reference);
    frame->width = frame_width;
    frame->height = frame_height;
    frame->rgb565 = pixels;
    frame->handle = pixels;
    return true;
}

void hal_camera_release(cam_frame_t *frame)
{
    free(frame->handle);
}

/* ---- motion: write G-code to a file ---- */
bool hal_motion_send(const char *line, uint32_t timeout_ms)
{
    (void)timeout_ms;
    fprintf(s_gcode_file, "%s\n", line);
    s_gcode_lines++;
    return true;
}

/* ---- operator ---- */
void hal_wait_button(const char *prompt)
{
    hal_log("[button] %s  (auto-continue in sim)\n", prompt);
    fprintf(s_gcode_file, "; PAUSE: %s\n", prompt);
}

bool hal_calibrate_requested(void)
{
    return s_calibrate;
}

void hal_debug_dump_rgb(const char *name, int width, int height, const uint8_t *rgb)
{
    char path[512];
    snprintf(path, sizeof path, "%s/%s.ppm", s_out_dir, name);
    FILE *file = fopen(path, "wb");
    if (!file)
    {
        return;
    }
    fprintf(file, "P6\n%d %d\n255\n", width, height);
    fwrite(rgb, 3, (size_t)width * height, file);
    fclose(file);
    hal_log("host: wrote %s  (%ld G-code lines so far)\n", path, s_gcode_lines);
}
