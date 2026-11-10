/*
 * hal_esp32.c - Seeed Studio XIAO ESP32S3 Sense (ESP32-S3R8: 8 MB flash, 8 MB PSRAM, OV2640).
 *
 * Pins used (XIAO silkscreen):
 *   D6 / GPIO43  UART1 TX  -> motion board RX  (SKR Mini E3 V3 "TFT" header)
 *   D7 / GPIO44  UART1 RX  <- motion board TX
 *   D1 / GPIO2   button to GND (internal pull-up)
 *   GPIO21       on-board user LED (active low)
 *   camera       on the Sense expansion board (fixed pins below)
 * Console/logging goes over the USB-C port (USB-Serial-JTAG).
 */
#ifdef ESP_PLATFORM

#include "config.h"
#include "hal/hal.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_camera.h"
#include "esp_rom_crc.h"
#include "esp_heap_caps.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MOTION_UART   UART_NUM_1
#define MOTION_TX     43
#define MOTION_RX     44
#define MOTION_BAUD   115200
#define BUTTON_GPIO   GPIO_NUM_2
#define LED_GPIO      GPIO_NUM_21

/* survives panic / watchdog / brownout resets, garbage after power-on */
static RTC_NOINIT_ATTR uint32_t s_boot_counter;

void hal_init(void) {
    gpio_config_t in = { .pin_bit_mask = 1ULL << BUTTON_GPIO, .mode = GPIO_MODE_INPUT,
                         .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&in);
    gpio_config_t out = { .pin_bit_mask = 1ULL << LED_GPIO, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&out);
    hal_led(false);

    uart_config_t uc = { .baud_rate = MOTION_BAUD, .data_bits = UART_DATA_8_BITS,
                         .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
                         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT };
    ESP_ERROR_CHECK(uart_driver_install(MOTION_UART, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MOTION_UART, &uc));
    ESP_ERROR_CHECK(uart_set_pin(MOTION_UART, MOTION_TX, MOTION_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

uint32_t hal_millis(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
void hal_delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1)); }

void hal_log(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

void *hal_alloc_large(size_t n) { return heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); }
void hal_free_large(void *p) { heap_caps_free(p); }

/* ---------------- camera ---------------- */
static bool s_exposure_locked;

bool hal_camera_init(void) {
    camera_config_t c = {
        .pin_pwdn = -1, .pin_reset = -1, .pin_xclk = 10,
        .pin_sccb_sda = 40, .pin_sccb_scl = 39,
        .pin_d7 = 48, .pin_d6 = 11, .pin_d5 = 12, .pin_d4 = 14,
        .pin_d3 = 16, .pin_d2 = 18, .pin_d1 = 17, .pin_d0 = 15,
        .pin_vsync = 38, .pin_href = 47, .pin_pclk = 13,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0, .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_RGB565,
        .frame_size = FRAMESIZE_SVGA,           /* 800x600 RGB565 = 960 KB in PSRAM */
        .fb_count = 1,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST,
    };
    if (esp_camera_init(&c) != ESP_OK) return false;
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 0);
    s->set_hmirror(s, 0);
    return true;
}

bool hal_camera_capture(capture_purpose_t why, cam_frame_t *out) {
    sensor_t *s = esp_camera_sensor_get();
    /* Canvas photos must be comparable to each other: freeze AE/AWB on the
     * first one so exposure drift isn't mistaken for paint. */
    if (why == CAPTURE_CANVAS && !s_exposure_locked) {
        for (int i = 0; i < 8; i++) { camera_fb_t *w = esp_camera_fb_get(); if (w) esp_camera_fb_return(w); }
        s->set_exposure_ctrl(s, 0);
        s->set_gain_ctrl(s, 0);
        s->set_whitebal(s, 0);
        s_exposure_locked = true;
    }
    /* discard stale frames so the image is taken now, not before the gantry parked */
    for (int i = 0; i < 3; i++) { camera_fb_t *w = esp_camera_fb_get(); if (w) esp_camera_fb_return(w); }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return false;
    /* esp32-camera delivers RGB565 big-endian; make it native (little-endian) */
    uint16_t *px = (uint16_t *)fb->buf;
    size_t n = fb->len / 2;
    for (size_t i = 0; i < n; i++) px[i] = (uint16_t)((px[i] >> 8) | (px[i] << 8));
    out->w = fb->width; out->h = fb->height; out->rgb565 = px; out->handle = fb;
    return true;
}

void hal_camera_release(cam_frame_t *f) { esp_camera_fb_return((camera_fb_t *)f->handle); }

/* ---------------- motion board (Marlin) ---------------- */
bool hal_motion_send(const char *line, uint32_t timeout_ms) {
    uart_write_bytes(MOTION_UART, line, strlen(line));
    uart_write_bytes(MOTION_UART, "\n", 1);

    char buf[128];
    size_t len = 0;
    uint32_t deadline = hal_millis() + timeout_ms;
    while ((int32_t)(deadline - hal_millis()) > 0) {
        uint8_t ch;
        if (uart_read_bytes(MOTION_UART, &ch, 1, pdMS_TO_TICKS(20)) != 1) continue;
        if (ch != '\n') { if (len < sizeof buf - 1) buf[len++] = (char)ch; continue; }
        buf[len] = 0;
        if (len && buf[len - 1] == '\r') buf[--len] = 0;
        if (!strncmp(buf, "ok", 2)) return true;
        if (!strncmp(buf, "Error", 5) || !strncmp(buf, "!!", 2)) { hal_log("marlin: %s\n", buf); return false; }
        if (!strncmp(buf, "echo:busy", 9)) deadline = hal_millis() + timeout_ms;  /* still working */
        len = 0;
    }
    hal_log("marlin: timeout on '%s'\n", line);
    return false;
}

/* ---------------- operator ---------------- */
bool hal_button_held(void) { return gpio_get_level(BUTTON_GPIO) == 0; }

void hal_led(bool on) { gpio_set_level(LED_GPIO, on ? 0 : 1); }

void hal_halt(void) {
    for (;;) { hal_led(true); hal_delay_ms(150); hal_led(false); hal_delay_ms(150); }
}

void hal_wait_button(const char *prompt) {
    hal_log("\n>>> %s\n", prompt);
    while (hal_button_held()) hal_delay_ms(20);              /* wait for release first */
    for (uint32_t t = 0; !hal_button_held(); t++) { hal_led((t / 25) & 1); hal_delay_ms(20); }
    hal_delay_ms(30);
    while (hal_button_held()) hal_delay_ms(20);
    hal_led(false);
}

/* ---------------- storage: raw "storage" partition, fixed slot offsets ---------------- */
typedef struct { uint32_t magic, len, crc; } slot_hdr_t;
#define SLOT_HDR_MAGIC 0x51077A6Eu
static const uint32_t k_slot_off[4] = { 0x00000, 0x40000, 0x50000, 0x60000 };
static const uint32_t k_slot_cap[4] = { 0x40000, 0x10000, 0x10000, 0x10000 };

static const esp_partition_t *storage(void) {
    static const esp_partition_t *p;
    if (!p) p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    return p;
}

bool hal_storage_save(int slot, const void *data, size_t len) {
    const esp_partition_t *p = storage();
    if (!p || slot < 0 || slot > 3 || len + sizeof(slot_hdr_t) > k_slot_cap[slot]) return false;
    size_t erase = (len + sizeof(slot_hdr_t) + 4095) & ~4095u;
    slot_hdr_t h = { SLOT_HDR_MAGIC, (uint32_t)len, esp_rom_crc32_le(0, (const uint8_t *)data, (uint32_t)len) };
    return esp_partition_erase_range(p, k_slot_off[slot], erase) == ESP_OK &&
           esp_partition_write(p, k_slot_off[slot] + sizeof h, data, len) == ESP_OK &&
           esp_partition_write(p, k_slot_off[slot], &h, sizeof h) == ESP_OK;  /* header last = atomic-ish */
}

bool hal_storage_load(int slot, void *data, size_t len) {
    const esp_partition_t *p = storage();
    slot_hdr_t h;
    if (!p || slot < 0 || slot > 3) return false;
    if (esp_partition_read(p, k_slot_off[slot], &h, sizeof h) != ESP_OK) return false;
    if (h.magic != SLOT_HDR_MAGIC || h.len != len) return false;
    if (esp_partition_read(p, k_slot_off[slot] + sizeof h, data, len) != ESP_OK) return false;
    return esp_rom_crc32_le(0, (const uint8_t *)data, (uint32_t)len) == h.crc;
}

/* ---------------- boot ---------------- */
reset_reason_t hal_reset_reason(void) {
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return RESET_POWERON;
    case ESP_RST_SW: return RESET_SOFTWARE;
    case ESP_RST_PANIC: return RESET_PANIC;
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: return RESET_WATCHDOG;
    case ESP_RST_BROWNOUT: return RESET_BROWNOUT;
    default: return RESET_OTHER;
    }
}
uint32_t hal_boot_counter_get(void) { return s_boot_counter > 1000 ? 0 : s_boot_counter; }
void hal_boot_counter_set(uint32_t v) { s_boot_counter = v; }
void hal_boot_mark_app_valid(void) { esp_ota_mark_app_valid_cancel_rollback(); }

void hal_debug_dump_rgb(const char *name, int w, int h, const uint8_t *rgb) {
    (void)rgb;
    hal_log("debug: %s (%dx%d) - image dump over serial not implemented\n", name, w, h);
}

#endif /* ESP_PLATFORM */
