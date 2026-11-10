#include "board.h"
#include "stm32f4.h"

#define LED_PIN 5u /* PA5 */

void board_init(void)
{
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC_AHB1ENR; /* let the clock settle (RM0368 errata) */
    GPIOA->MODER = (GPIOA->MODER & ~(3u << (LED_PIN * 2))) | (GPIO_MODE_OUTPUT << (LED_PIN * 2));
    led_set(false);
}

void led_set(bool on)
{
    GPIOA->BSRR = on ? (1u << LED_PIN) : (1u << (LED_PIN + 16));
}

void led_toggle(void)
{
    led_set(!(GPIOA->ODR & (1u << LED_PIN)));
}
