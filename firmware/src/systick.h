#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

void systick_init(void); /* 1 ms tick; also wakes the main loop from WFI */
uint32_t millis(void);

#endif
