/*
 * hal.h - the only interface between portable code and hardware.
 * Implementations: hal/esp32/hal_esp32.c (XIAO ESP32S3 Sense),
 *                  hal/host/hal_host.c   (laptop simulator).
 */
#ifndef LEOGOYA_HAL_H
#define LEOGOYA_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum { CAPTURE_REFERENCE, CAPTURE_CANVAS } capture_purpose_t;

typedef struct {
    int w, h;
    const uint16_t *rgb565;  /* row-major, w*h pixels, native-endian RGB565 */
    void *handle;            /* port-private */
} cam_frame_t;

typedef enum {
    RESET_POWERON, RESET_SOFTWARE, RESET_PANIC, RESET_WATCHDOG,
    RESET_BROWNOUT, RESET_OTHER
} reset_reason_t;

/* lifecycle */
void hal_init(void);
uint32_t hal_millis(void);
void hal_delay_ms(uint32_t ms);
void hal_log(const char *fmt, ...);

/* memory: large buffers go to PSRAM on the ESP32 */
void *hal_alloc_large(size_t bytes);
void hal_free_large(void *p);

/* camera */
bool hal_camera_init(void);
bool hal_camera_capture(capture_purpose_t why, cam_frame_t *out);
void hal_camera_release(cam_frame_t *f);

/* motion board link: send one G-code line, block until "ok" (true) or error/timeout */
bool hal_motion_send(const char *line, uint32_t timeout_ms);

/* operator I/O: a single button + status LED */
bool hal_button_held(void);
void hal_wait_button(const char *prompt);   /* logs prompt, blocks until press */
void hal_led(bool on);
void hal_halt(void);                        /* unrecoverable: stop forever (sim: exit) */

/* persistent storage: small numbered slots that survive power loss */
bool hal_storage_save(int slot, const void *data, size_t len);
bool hal_storage_load(int slot, void *data, size_t len);

/* boot support */
reset_reason_t hal_reset_reason(void);
uint32_t hal_boot_counter_get(void);
void hal_boot_counter_set(uint32_t v);
void hal_boot_mark_app_valid(void);         /* cancels OTA rollback */

/* host-only debugging output; no-op on hardware */
void hal_debug_dump_rgb(const char *name, int w, int h, const uint8_t *rgb);

enum { SLOT_REFERENCE = 0, SLOT_PALETTE = 1, SLOT_BRUSH = 2, SLOT_PROGRESS = 3 };

#endif
