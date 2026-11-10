#include "cmd.h"
#include "board.h"
#include "fmt.h"
#include "gcode.h"
#include "stm32f4.h"
#include "systick.h"
#include "uart.h"
#include "version.h"

#include <stdbool.h>

#define LINE_MAX         96 /* bytes per command line, including the terminator */
#define MAX_ARGS         8
#define ASCII_BACKSPACE  0x08
#define ASCII_DELETE     0x7F  /* what most terminals send for the backspace key */
#define UART_DRAIN_LOOPS 20000 /* busy-wait so "ok" leaves the UART before RESET */

static char s_line_buf[LINE_MAX];
static uint32_t s_line_len;
static bool s_too_long;
static bool s_in_comment; /* after ';' until end of line: not stored, so any length is fine */
static bool s_echo;       /* local echo for humans in a terminal */
extern bool g_led_auto;   /* main.c: heartbeat blink */

typedef bool (*handler_t)(int argc, char **argv); /* return false -> "error: bad arguments" */

/* ---------------- commands ---------------- */

static bool handle_help(int argc, char **argv);

static bool handle_ping(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    uart_puts("pong");
    out_nl();
    return true;
}

static bool handle_version(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    uart_puts("leogoya-fw " FW_VERSION " (" BOARD_NAME ")");
    out_nl();
    return true;
}

static bool handle_status(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    uart_puts("uptime_ms ");
    out_u32(millis());
    out_nl();
    uart_puts("rx_dropped ");
    out_u32(uart_rx_dropped());
    out_nl();
    gcode_report_position();
    return true;
}

static bool handle_led(int argc, char **argv)
{
    if (argc != 2)
    {
        return false;
    }
    if (str_ieq(argv[1], "on"))
    {
        g_led_auto = false;
        led_set(true);
    }
    else if (str_ieq(argv[1], "off"))
    {
        g_led_auto = false;
        led_set(false);
    }
    else if (str_ieq(argv[1], "toggle"))
    {
        g_led_auto = false;
        led_toggle();
    }
    else if (str_ieq(argv[1], "auto"))
    {
        g_led_auto = true;
    }
    else
    {
        return false;
    }
    return true;
}

static bool handle_echo(int argc, char **argv)
{
    if (argc != 2)
    {
        return false;
    }
    if (str_ieq(argv[1], "on"))
    {
        s_echo = true;
    }
    else if (str_ieq(argv[1], "off"))
    {
        s_echo = false;
    }
    else
    {
        return false;
    }
    return true;
}

static bool handle_reset(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    uart_puts("ok");
    out_nl();
    for (volatile int i = 0; i < UART_DRAIN_LOOPS; i++)
    {
    }
    system_reset();
    return true;
}

typedef struct
{
    const char *name;
    handler_t handler;
    const char *usage;
} command_t;

static const command_t k_commands[] = {
    { "HELP", handle_help, "HELP                     list commands" },
    { "PING", handle_ping, "PING                     replies pong" },
    { "VERSION", handle_version, "VERSION                  firmware version + board" },
    { "STATUS", handle_status, "STATUS                   uptime, dropped bytes, position" },
    { "LED", handle_led, "LED ON|OFF|TOGGLE|AUTO   user LED (AUTO = heartbeat)" },
    { "ECHO", handle_echo, "ECHO ON|OFF              echo typed characters (for terminals)" },
    { "RESET", handle_reset, "RESET                    reboot the MCU" },
};
#define NUM_COMMANDS (sizeof k_commands / sizeof k_commands[0])

static bool handle_help(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    for (uint32_t i = 0; i < NUM_COMMANDS; i++)
    {
        uart_puts(k_commands[i].usage);
        out_nl();
    }
    gcode_help();
    return true;
}

/* ---------------- line handling ---------------- */

static bool is_space(char ch)
{
    return ch == ' ' || ch == '\t';
}

static bool is_backspace(uint8_t byte)
{
    return byte == ASCII_BACKSPACE || byte == ASCII_DELETE;
}

/* Split a line in place into whitespace-separated words (argv style). */
static int split_args(char *line, char **argv)
{
    int argc = 0;
    char *cursor = line;
    while (*cursor && argc < MAX_ARGS)
    {
        while (is_space(*cursor))
        {
            *cursor++ = 0;
        }
        if (!*cursor)
        {
            break;
        }
        argv[argc++] = cursor;
        while (*cursor && !is_space(*cursor))
        {
            cursor++;
        }
    }
    return argc;
}

static void reply_error(const char *why)
{
    uart_puts("error: ");
    uart_puts(why);
    out_nl();
}

static void run_line(char *line)
{
    char *argv[MAX_ARGS];
    int argc = split_args(line, argv);
    if (argc == 0)
    {
        return; /* blank line: no reply */
    }

    char first_letter = ascii_upper(argv[0][0]);
    bool is_gcode = (first_letter == 'G' || first_letter == 'M') && argv[0][1] >= '0' && argv[0][1] <= '9';
    if (is_gcode)
    {
        const char *error = gcode_execute(argc, argv);
        if (error)
        {
            reply_error(error);
        }
        else
        {
            uart_puts("ok");
            out_nl();
        }
        return;
    }
    for (uint32_t i = 0; i < NUM_COMMANDS; i++)
    {
        if (str_ieq(argv[0], k_commands[i].name))
        {
            if (k_commands[i].handler(argc, argv))
            {
                uart_puts("ok");
                out_nl();
            }
            else
            {
                uart_puts("usage: ");
                uart_puts(k_commands[i].usage);
                out_nl();
                reply_error("bad arguments");
            }
            return;
        }
    }
    reply_error("unknown command (try HELP)");
}

void cmd_feed(uint8_t byte)
{
    if (byte == '\r')
    {
        return;
    }
    if (s_echo)
    {
        if (byte == '\n')
        {
            out_nl();
        }
        else if (is_backspace(byte))
        {
            uart_puts("\b \b");
        }
        else
        {
            uart_putc((char)byte);
        }
    }
    if (byte == '\n')
    {
        if (s_too_long)
        {
            reply_error("line too long");
        }
        else
        {
            s_line_buf[s_line_len] = 0;
            run_line(s_line_buf);
        }
        s_line_len = 0;
        s_too_long = false;
        s_in_comment = false;
        return;
    }
    if (byte == ';')
    {
        s_in_comment = true;
    }
    if (s_in_comment)
    {
        return;
    }
    if (is_backspace(byte))
    {
        if (s_line_len)
        {
            s_line_len--;
        }
        return;
    }
    if (s_line_len < LINE_MAX - 1)
    {
        s_line_buf[s_line_len++] = (char)byte;
    }
    else
    {
        s_too_long = true;
    }
}
