/*
 * board.h - Nucleo-F401RE (also works unchanged on Nucleo-F411RE).
 *
 *   Serial : USART2, PA2 = TX, PA3 = RX (AF7), wired to the on-board ST-LINK,
 *            so it shows up on your computer as a USB serial port
 *            (macOS: /dev/cu.usbmodem*). No extra wiring needed.
 *            To use an external USB-serial adapter instead, connect to
 *            CN10 pins 35 (PA2) / 37 (PA3) after removing solder bridges
 *            SB13/SB14 (see UM1724).
 *   LED    : LD2 on PA5 (active high)
 *   Clock  : 16 MHz HSI, the reset default (no PLL). Plenty for a command loop.
 */
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
