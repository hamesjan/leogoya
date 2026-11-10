#ifndef GCODE_H
#define GCODE_H

const char *gcode_execute(int argc, char **argv); /* NULL = ok, else error text */
void gcode_report_position(void);
void gcode_help(void);

#endif
