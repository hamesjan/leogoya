/*
 * main.c - simulator entry point (runs on your laptop).
 * Arguments: see hal/host/hal_host.c.
 */
#include "app/app.h"

void hal_host_args(int argc, char **argv);

int main(int argc, char **argv)
{
    hal_host_args(argc, argv);
    app_run();
    return 0;
}
