#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stdint.h>

#define BOARD_NAME "Nucleo-F401RE"
#define SYSCLK_HZ  16000000u
#define UART_BAUD  115200u

void board_init(void);
void led_set(bool on);
void led_toggle(void);

#endif
