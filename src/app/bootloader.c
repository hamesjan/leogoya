/*
 * bootloader.c - application-side boot policy.
 *
 * The ESP32-S3 already has a ROM bootloader + ESP-IDF 2nd-stage bootloader
 * (selects the OTA slot, verifies the image). Writing a custom bootloader is
 * not worth it; this file is the policy that runs first in the app:
 *
 *   - crash-loop detection: N abnormal resets in a row -> SAFE mode, no motion
 *     (a firmware bug that panics mid-stroke must not keep driving a wet brush)
 *   - OTA rollback: a new image is confirmed only after it homes successfully;
 *     if it crashes before that, the IDF bootloader rolls back to the old slot
 *     (needs CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE, see sdkconfig.defaults)
 *   - mode select: hold the button at power-on to enter brush calibration
 */
#include "app/bootloader.h"
#include "config.h"
#include "hal/hal.h"

static bool abnormal(reset_reason_t r) {
    return r == RESET_PANIC || r == RESET_WATCHDOG || r == RESET_BROWNOUT;
}

boot_mode_t boot_init(void) {
    reset_reason_t why = hal_reset_reason();
    uint32_t crashes = hal_boot_counter_get();

    crashes = abnormal(why) ? crashes + 1 : 0;
    hal_boot_counter_set(crashes);

    if (why == RESET_BROWNOUT)
        hal_log("boot: brownout - steppers and MCU on the same weak supply? see docs/WIRING.md\n");

    if (crashes >= BOOT_CRASH_LIMIT) {
        hal_log("boot: %u abnormal resets in a row -> SAFE mode\n", (unsigned)crashes);
        return BOOT_SAFE;
    }

    /* debounce: button must be held for ~300 ms */
    int held = 0;
    for (int i = 0; i < 6; i++) { held += hal_button_held(); hal_delay_ms(50); }
    return held >= 5 ? BOOT_CALIBRATE : BOOT_NORMAL;
}

void boot_mark_healthy(void) {
    hal_boot_counter_set(0);
    hal_boot_mark_app_valid();
}

const char *boot_mode_name(boot_mode_t m) {
    switch (m) {
    case BOOT_NORMAL: return "normal";
    case BOOT_CALIBRATE: return "calibrate";
    case BOOT_SAFE: return "safe";
    }
    return "?";
}
