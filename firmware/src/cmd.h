/*
 * cmd.h - line-based serial command protocol.
 *
 * Wire format (115200 8N1):
 *   - One command per line, terminated by '\n' ('\r' ignored), max 95 chars.
 *   - Case-insensitive. Anything after ';' is a comment.
 *   - Every non-empty line gets zero or more info lines, then exactly ONE
 *     final line: "ok" or "error: <reason>". A sender should wait for that
 *     final line before sending the next command (same contract as Marlin,
 *     so the planner's G-code streams straight in).
 *
 * Commands: see the table in cmd.c, or send HELP.
 */
#ifndef CMD_H
#define CMD_H

#include <stdint.h>

void cmd_feed(uint8_t byte); /* feed received bytes; runs a command per complete line */

#endif
