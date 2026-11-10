#ifndef FMT_H
#define FMT_H

#include <stdbool.h>
#include <stdint.h>

void out_u32(uint32_t value);
void out_fixed(float value, int decimals); /* e.g. out_fixed(-1.5f, 3) -> "-1.500" */
void out_nl(void);

bool parse_float(const char *text, float *result); /* whole string must be a number */
bool str_ieq(const char *a, const char *b);        /* case-insensitive equal */
char ascii_upper(char ch);

#endif
