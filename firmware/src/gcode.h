/*
 * gcode.h - the subset of G-code the planner (planner/motion) emits.
 *
 * No motors are driven yet: moves update a commanded position so the protocol
 * can be tested end to end. Step generation plugs in at gcode_move().
 */
#ifndef GCODE_H
#define GCODE_H

const char *gcode_execute(int argc, char **argv); /* NULL = ok, else error text */
void gcode_report_position(void);
void gcode_help(void);

#endif
