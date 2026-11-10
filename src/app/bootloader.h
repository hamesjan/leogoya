#ifndef LEOGOYA_BOOTLOADER_H
#define LEOGOYA_BOOTLOADER_H

typedef enum {
    BOOT_NORMAL,      /* paint (or resume) */
    BOOT_CALIBRATE,   /* button held at power-on: brush calibration */
    BOOT_SAFE,        /* repeated crashes: no motion, wait for operator */
} boot_mode_t;

boot_mode_t boot_init(void);
void boot_mark_healthy(void);   /* call once the app is known-good (homed OK) */
const char *boot_mode_name(boot_mode_t m);

#endif
