#ifndef UART_H
#define UART_H

#include <stdbool.h>
#include <stdint.h>

void uart_init(void);
bool uart_getc(uint8_t *byte); /* non-blocking: false if nothing received */
void uart_putc(char ch);       /* blocking */
void uart_puts(const char *text);
uint32_t uart_rx_dropped(void); /* bytes lost to a full buffer or overrun */

#endif
