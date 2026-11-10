#ifndef CMD_H
#define CMD_H

#include <stdint.h>

void cmd_feed(uint8_t byte); /* feed received bytes; runs a command per complete line */

#endif
