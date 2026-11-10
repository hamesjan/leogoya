/*
 * stm32f4.h - the few STM32F401/F411 registers this firmware touches.
 * Addresses and bit positions: RM0368 (STM32F401 reference manual).
 * Kept self-contained so the build needs nothing but arm-none-eabi-gcc.
 * Porting to another STM32 family = rewrite this file + board.h.
 */
#ifndef STM32F4_H
#define STM32F4_H

#include <stdint.h>

#define REG(addr) (*(volatile uint32_t *)(addr))

/* ---- RCC ---- */
#define RCC_BASE             0x40023800u
#define RCC_AHB1ENR          REG(RCC_BASE + 0x30)
#define RCC_APB1ENR          REG(RCC_BASE + 0x40)
#define RCC_AHB1ENR_GPIOAEN  (1u << 0)
#define RCC_APB1ENR_USART2EN (1u << 17)

/* ---- GPIO ---- */
typedef struct
{
    volatile uint32_t MODER;   /* 0x00 mode: input / output / alternate / analog */
    volatile uint32_t OTYPER;  /* 0x04 output type */
    volatile uint32_t OSPEEDR; /* 0x08 output speed */
    volatile uint32_t PUPDR;   /* 0x0C pull-up / pull-down */
    volatile uint32_t IDR;     /* 0x10 input data */
    volatile uint32_t ODR;     /* 0x14 output data */
    volatile uint32_t BSRR;    /* 0x18 atomic bit set / reset */
    volatile uint32_t LCKR;    /* 0x1C configuration lock */
    volatile uint32_t AFR[2];  /* 0x20 alternate function select, pins 0-7 and 8-15 */
} gpio_t;
#define GPIOA ((gpio_t *)0x40020000u)

#define GPIO_MODE_OUTPUT 1u
#define GPIO_MODE_AF     2u

/* ---- USART ---- */
typedef struct
{
    volatile uint32_t SR;   /* 0x00 status */
    volatile uint32_t DR;   /* 0x04 data */
    volatile uint32_t BRR;  /* 0x08 baud rate */
    volatile uint32_t CR1;  /* 0x0C control 1 */
    volatile uint32_t CR2;  /* 0x10 control 2 */
    volatile uint32_t CR3;  /* 0x14 control 3 */
    volatile uint32_t GTPR; /* 0x18 guard time / prescaler */
} usart_t;
#define USART2           ((usart_t *)0x40004400u)
#define USART_SR_ORE     (1u << 3)
#define USART_SR_RXNE    (1u << 5)
#define USART_SR_TXE     (1u << 7)
#define USART_CR1_RE     (1u << 2)
#define USART_CR1_TE     (1u << 3)
#define USART_CR1_RXNEIE (1u << 5)
#define USART_CR1_UE     (1u << 13)
#define USART2_IRQn      38

/* ---- Cortex-M4 core ---- */
#define SYST_CSR     REG(0xE000E010u)
#define SYST_RVR     REG(0xE000E014u)
#define SYST_CVR     REG(0xE000E018u)
#define NVIC_ISER(n) REG(0xE000E100u + 4u * (n))
#define SCB_AIRCR    REG(0xE000ED0Cu)
#define SCB_CPACR    REG(0xE000ED88u)

static inline void nvic_enable(int irq)
{
    NVIC_ISER(irq >> 5) = 1u << (irq & 31);
}
static inline void irq_enable(void)
{
    __asm volatile("cpsie i" ::: "memory");
}
static inline void wfi(void)
{
    __asm volatile("wfi");
}

static inline void system_reset(void)
{
    __asm volatile("dsb" ::: "memory");
    SCB_AIRCR = (0x05FAu << 16) | (1u << 2); /* VECTKEY | SYSRESETREQ */
    for (;;)
    {
    }
}

#endif
