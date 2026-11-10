/*
 * hal.h - the only interface between the simulator and the outside world.
 * Implementation: hal/host/hal_host.c (runs on your laptop: reference image
 * from a file, G-code to a file). A serial port to the STM32 controller can be
 * added later as another hal_motion_send() backend.
 */
#ifndef LEOGOYA_HAL_H
#define LEOGOYA_HAL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    CAPTURE_REFERENCE,
    CAPTURE_CANVAS
} capture_purpose_t;

typedef struct
{
    int width;
    int height;
    const uint16_t *rgb565; /* row-major, width*height pixels, native-endian RGB565 */
    void *handle;           /* owned by the HAL implementation */
} cam_frame_t;

/* lifecycle */
void hal_init(void);
uint32_t hal_millis(void);
void hal_log(const char *fmt, ...);

/* camera */
bool hal_camera_init(void);
bool hal_camera_capture(capture_purpose_t purpose, cam_frame_t *frame);
void hal_camera_release(cam_frame_t *frame);

/* motion controller link: send one G-code line, block until "ok" (true) or error/timeout */
bool hal_motion_send(const char *line, uint32_t timeout_ms);

/* operator prompt: blocks until the operator confirms (logged + written into the G-code) */
void hal_wait_button(const char *prompt);

/* run the brush calibration routine instead of a normal painting */
bool hal_calibrate_requested(void);

/* debugging output: preview images */
void hal_debug_dump_rgb(const char *name, int width, int height, const uint8_t *rgb);

#endif
