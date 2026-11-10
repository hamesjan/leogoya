/*
 * uart.c - USART2 at 115200 8N1.
 * RX: interrupt pushes into a ring buffer (ISR is the only writer of head,
 *     main loop the only writer of tail, so no locking is needed).
 * TX: blocking on TXE. ~87 us per byte; replies are short.
 */
#include "uart.h"
#include "board.h"
#include "stm32f4.h"

#define RX_BUF_SIZE 512u /* power of two */

static volatile uint8_t s_rx_buf[RX_BUF_SIZE];
static volatile uint32_t s_rx_head;    /* next slot the ISR writes */
static volatile uint32_t s_rx_tail;    /* next slot the main loop reads */
static volatile uint32_t s_rx_dropped; /* bytes lost: buffer full or hardware overrun */

void uart_init(void)
{
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC_APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC_APB1ENR;

    /* PA2, PA3 -> alternate function 7 (USART2), RX with pull-up */
    GPIOA->MODER = (GPIOA->MODER & ~(0xFu << 4)) | ((GPIO_MODE_AF << 4) | (GPIO_MODE_AF << 6));
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFFu << 8)) | (7u << 8) | (7u << 12);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3u << 6)) | (1u << 6);

    USART2->BRR = (SYSCLK_HZ + UART_BAUD / 2) / UART_BAUD; /* oversampling by 16 */
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    nvic_enable(USART2_IRQn);
}

void USART2_IRQHandler(void)
{
    uint32_t status = USART2->SR;
    if (status & (USART_SR_RXNE | USART_SR_ORE))
    {
        uint8_t byte = (uint8_t)USART2->DR; /* reading SR then DR also clears ORE */
        if (status & USART_SR_ORE)
        {
            s_rx_dropped++;
        }
        uint32_t next_head = (s_rx_head + 1) & (RX_BUF_SIZE - 1);
        if (next_head == s_rx_tail)
        {
            s_rx_dropped++;
            return;
        }
        s_rx_buf[s_rx_head] = byte;
        s_rx_head = next_head;
    }
}

bool uart_getc(uint8_t *byte)
{
    if (s_rx_tail == s_rx_head)
    {
        return false;
    }
    *byte = s_rx_buf[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1) & (RX_BUF_SIZE - 1);
    return true;
}

void uart_putc(char ch)
{
    while (!(USART2->SR & USART_SR_TXE))
    {
        /* wait for the transmit register to empty */
    }
    USART2->DR = (uint8_t)ch;
}

void uart_puts(const char *text)
{
    while (*text)
    {
        uart_putc(*text++);
    }
}

uint32_t uart_rx_dropped(void)
{
    return s_rx_dropped;
}
