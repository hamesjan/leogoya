#include "fmt.h"
#include "uart.h"

void out_nl(void)
{
    uart_puts("\r\n");
}

void out_u32(uint32_t value)
{
    char digits[11]; /* 4294967295 + terminator */
    int first = 10;  /* digits are written right to left */
    digits[first] = 0;
    do
    {
        digits[--first] = (char)('0' + value % 10);
        value /= 10;
    } while (value);
    uart_puts(&digits[first]);
}

void out_fixed(float value, int decimals)
{
    if (value < 0)
    {
        uart_putc('-');
        value = -value;
    }
    uint32_t scale = 1; /* 10^decimals */
    for (int i = 0; i < decimals; i++)
    {
        scale *= 10;
    }
    uint32_t scaled = (uint32_t)(value * (float)scale + 0.5f); /* rounded, e.g. 1.5 @ 3 -> 1500 */
    out_u32(scaled / scale);
    if (decimals > 0)
    {
        uart_putc('.');
        uint32_t fraction = scaled % scale;
        for (uint32_t place = scale / 10; place > 0; place /= 10) /* leading zeros included */
        {
            uart_putc((char)('0' + fraction / place));
            fraction %= place;
        }
    }
}

bool parse_float(const char *text, float *result)
{
    float sign = 1.0f;
    float value = 0.0f;
    float place = 0.0f; /* 0 before the decimal point, then 0.1, 0.01, ... */
    int digit_count = 0;
    if (*text == '-' || *text == '+')
    {
        if (*text == '-')
        {
            sign = -1.0f;
        }
        text++;
    }
    for (; *text; text++)
    {
        if (*text >= '0' && *text <= '9')
        {
            float digit = (float)(*text - '0');
            if (place > 0.0f)
            {
                value += digit * place;
                place *= 0.1f;
            }
            else
            {
                value = value * 10.0f + digit;
            }
            digit_count++;
        }
        else if (*text == '.' && place == 0.0f)
        {
            place = 0.1f;
        }
        else
        {
            return false;
        }
    }
    if (!digit_count)
    {
        return false;
    }
    *result = sign * value;
    return true;
}

char ascii_upper(char ch)
{
    return (ch >= 'a' && ch <= 'z') ? (char)(ch - 'a' + 'A') : ch;
}

bool str_ieq(const char *a, const char *b)
{
    while (*a && *b)
    {
        if (ascii_upper(*a++) != ascii_upper(*b++))
        {
            return false;
        }
    }
    return *a == *b;
}
