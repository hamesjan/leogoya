/*
 * startup.c - vector table and reset handler for STM32F401/F411.
 */
#include "stm32f4.h"

#include <stdint.h>

/* Section boundaries, defined in stm32f401re.ld */
extern uint32_t __data_load_start; /* initialized globals: start values in flash */
extern uint32_t __data_start;      /* initialized globals: in RAM */
extern uint32_t __data_end;
extern uint32_t __bss_start; /* zero-initialized globals */
extern uint32_t __bss_end;
extern uint32_t __stack_top;
int main(void);

void Reset_Handler(void);
void Default_Handler(void);
void HardFault_Handler(void);
void SysTick_Handler(void);
void USART2_IRQHandler(void);

#define NUM_IRQS 86 /* F401/F411 vector table: 16 core + 86 peripheral slots */

/* Every slot defaults to Default_Handler; the ones we use are overridden below. */
/* clang-format off */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverride-init"
__attribute__((section(".isr_vector"), used))
void (*const g_vectors[16 + NUM_IRQS])(void) =
{
    [1 ... 16 + NUM_IRQS - 1] = Default_Handler,
    [0]  = (void (*)(void))&__stack_top,
    [1]  = Reset_Handler,
    [2]  = Default_Handler,      /* NMI */
    [3]  = HardFault_Handler,
    [4]  = Default_Handler,      /* MemManage */
    [5]  = Default_Handler,      /* BusFault */
    [6]  = Default_Handler,      /* UsageFault */
    [11] = Default_Handler,      /* SVCall */
    [12] = Default_Handler,      /* DebugMon */
    [14] = Default_Handler,      /* PendSV */
    [15] = SysTick_Handler,
    [16 + USART2_IRQn] = USART2_IRQHandler,
};
#pragma GCC diagnostic pop
/* clang-format on */

void Reset_Handler(void)
{
    SCB_CPACR |= 0xFu << 20; /* FPU on (CP10/CP11 full access) before any float code runs */
    __asm volatile("dsb\n isb");

    /* Initialized globals: copy their starting values from flash into RAM. */
    const uint32_t *initial_value = &__data_load_start;
    for (uint32_t *global = &__data_start; global < &__data_end; global++)
    {
        *global = *initial_value++;
    }

    /* Zero-initialized globals: clear them. */
    for (uint32_t *global = &__bss_start; global < &__bss_end; global++)
    {
        *global = 0;
    }

    main();
    for (;;)
    {
    }
}

/* Unused vectors land here. Stuck in a loop = unexpected interrupt; attach a debugger. */
void Default_Handler(void)
{
    for (;;)
    {
    }
}

void HardFault_Handler(void)
{
    for (;;)
    {
    }
}

/* Weak defaults so the table links even if a driver is left out. */
__attribute__((weak, alias("Default_Handler"))) void SysTick_Handler(void);
__attribute__((weak, alias("Default_Handler"))) void USART2_IRQHandler(void);
