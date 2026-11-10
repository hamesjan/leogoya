#include "systick.h"
#include "board.h"
#include "stm32f4.h"

static volatile uint32_t s_millis;

void systick_init(void)
{
    SYST_RVR = SYSCLK_HZ / 1000u - 1u; /* 1 kHz */
    SYST_CVR = 0;
    SYST_CSR = 7; /* CLKSOURCE = core, TICKINT, ENABLE */
}

uint32_t millis(void)
{
    return s_millis;
}

void SysTick_Handler(void)
{
    s_millis++;
}
