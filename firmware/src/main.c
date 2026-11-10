/*
 * main.c - Leogoya controller firmware: a main loop that waits for commands
 * on the serial wire and runs them. See cmd.h for the protocol.
 */
#include "board.h"
#include "cmd.h"
#include "fmt.h"
#include "stm32f4.h"
#include "systick.h"
#include "uart.h"
#include "version.h"

bool g_led_auto = true; /* heartbeat until a LED command takes over */

int main(void)
{
    board_init();
    systick_init();
    uart_init();
    irq_enable();

    out_nl();
    uart_puts("leogoya-fw " FW_VERSION " ready on " BOARD_NAME ", send HELP");
    out_nl();

    uint32_t last_blink_ms = 0;
    for (;;)
    {
        uint8_t byte;
        while (uart_getc(&byte))
        {
            cmd_feed(byte);
        }

        uint32_t now = millis();
        if (g_led_auto && now - last_blink_ms >= 500u)
        {
            last_blink_ms = now;
            led_toggle();
        }
        wfi(); /* sleep until the next byte (USART IRQ) or 1 ms tick (SysTick) */
    }
}
