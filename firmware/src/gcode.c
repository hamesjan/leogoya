#include "gcode.h"
#include "fmt.h"
#include "uart.h"

#include <stdbool.h>

enum
{
    AXIS_X,
    AXIS_Y,
    AXIS_Z,
    AXIS_A, /* brush yaw, degrees */
    NUM_AXES
};
static const char k_axis_letter[NUM_AXES] = { 'X', 'Y', 'Z', 'A' };

static float s_position[NUM_AXES];    /* commanded position: mm, mm, mm, degrees */
static float s_feed_mm_min = 1000.0f; /* last F word */
static bool s_relative_mode;          /* G91 active */

static int axis_index(char letter)
{
    letter = ascii_upper(letter);
    for (int axis = 0; axis < NUM_AXES; axis++)
    {
        if (k_axis_letter[axis] == letter)
        {
            return axis;
        }
    }
    return -1;
}

/* Parse the words after the code, e.g. "X12.5 Y3 F3000".
 * axis_value[] receives numbers, axis_given[] marks which axes appeared.
 * allow_bare_axes: axis letters may appear without a number (G28 X Y). */
static const char *parse_words(int argc, char **argv, float axis_value[NUM_AXES], bool axis_given[NUM_AXES],
                               float *feed_mm_min, bool allow_bare_axes)
{
    for (int axis = 0; axis < NUM_AXES; axis++)
    {
        axis_given[axis] = false;
    }
    for (int i = 1; i < argc; i++)
    {
        const char *word = argv[i];
        char letter = ascii_upper(word[0]);
        int axis = axis_index(letter);
        bool bare_axis = allow_bare_axes && axis >= 0 && word[1] == 0;
        float number = 0.0f;
        if (!bare_axis && !parse_float(word + 1, &number))
        {
            return "bad number";
        }
        if (axis >= 0)
        {
            axis_value[axis] = number;
            axis_given[axis] = true;
        }
        else if (letter == 'F' && feed_mm_min)
        {
            if (number <= 0)
            {
                return "bad feed";
            }
            *feed_mm_min = number;
        }
        else
        {
            return "unsupported word";
        }
    }
    return 0;
}

/* Hook for real motion: replace with step generation / a move queue. */
static void gcode_move(const float target[NUM_AXES], float feed_mm_min)
{
    (void)feed_mm_min;
    for (int axis = 0; axis < NUM_AXES; axis++)
    {
        s_position[axis] = target[axis];
    }
}

/* True if code_word is <letter><number>, e.g. is_code("g01", 'G', 1). */
static bool is_code(const char *code_word, char letter, int number)
{
    if (ascii_upper(code_word[0]) != letter)
    {
        return false;
    }
    float parsed;
    return parse_float(code_word + 1, &parsed) && parsed == (float)number;
}

const char *gcode_execute(int argc, char **argv)
{
    const char *code_word = argv[0];
    float axis_value[NUM_AXES];
    bool axis_given[NUM_AXES];
    const char *error;

    /* G0/G1: move */
    if (is_code(code_word, 'G', 0) || is_code(code_word, 'G', 1))
    {
        float feed_mm_min = s_feed_mm_min;
        if ((error = parse_words(argc, argv, axis_value, axis_given, &feed_mm_min, false)))
        {
            return error;
        }
        float target[NUM_AXES];
        for (int axis = 0; axis < NUM_AXES; axis++)
        {
            if (!axis_given[axis])
            {
                target[axis] = s_position[axis];
            }
            else if (s_relative_mode)
            {
                target[axis] = s_position[axis] + axis_value[axis];
            }
            else
            {
                target[axis] = axis_value[axis];
            }
        }
        s_feed_mm_min = feed_mm_min;
        gcode_move(target, feed_mm_min);
        return 0;
    }
    /* G28: home the given axes, or all of them */
    if (is_code(code_word, 'G', 28))
    {
        if ((error = parse_words(argc, argv, axis_value, axis_given, 0, true)))
        {
            return error;
        }
        bool any_axis_given = false;
        for (int axis = 0; axis < NUM_AXES; axis++)
        {
            any_axis_given |= axis_given[axis];
        }
        for (int axis = 0; axis < NUM_AXES; axis++)
        {
            if (!any_axis_given || axis_given[axis])
            {
                s_position[axis] = 0.0f;
            }
        }
        return 0;
    }
    /* G92: set position without moving */
    if (is_code(code_word, 'G', 92))
    {
        if ((error = parse_words(argc, argv, axis_value, axis_given, 0, false)))
        {
            return error;
        }
        for (int axis = 0; axis < NUM_AXES; axis++)
        {
            if (axis_given[axis])
            {
                s_position[axis] = axis_value[axis];
            }
        }
        return 0;
    }
    if (is_code(code_word, 'G', 90))
    {
        s_relative_mode = false;
        return 0;
    }
    if (is_code(code_word, 'G', 91))
    {
        s_relative_mode = true;
        return 0;
    }
    /* G21: millimetres, the only unit we support */
    if (is_code(code_word, 'G', 21))
    {
        return 0;
    }
    /* M400: wait for moves to finish; nothing is queued yet */
    if (is_code(code_word, 'M', 400))
    {
        return 0;
    }
    if (is_code(code_word, 'M', 114))
    {
        gcode_report_position();
        return 0;
    }
    return "unsupported G/M code";
}

void gcode_report_position(void)
{
    for (int i = 0; i < NUM_AXES; i++)
    {
        char label[3] = { k_axis_letter[i], ':', 0 };
        uart_puts(label);
        out_fixed(s_position[i], 3);
        uart_puts(i < NUM_AXES - 1 ? " " : "");
    }
    uart_puts(" F:");
    out_fixed(s_feed_mm_min, 0);
    uart_puts(s_relative_mode ? " REL" : " ABS");
    out_nl();
}

void gcode_help(void)
{
    uart_puts("G0/G1 [X Y Z A F]        move (no motors yet: updates position)");
    out_nl();
    uart_puts("G28 [X Y Z A]            home (zero) axes");
    out_nl();
    uart_puts("G92 [X Y Z A]            set position");
    out_nl();
    uart_puts("G90 / G91 / G21          absolute / relative / mm");
    out_nl();
    uart_puts("M114                     report position");
    out_nl();
    uart_puts("M400                     wait for moves to finish");
    out_nl();
}
