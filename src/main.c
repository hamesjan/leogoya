/*
 * main.c - entry point for both builds.
 *   ESP-IDF  : app_main() is called by the IDF startup code.
 *   host sim : main(argc, argv) -> see hal/host/hal_host.c for arguments.
 */
#include "app/app.h"

#ifdef ESP_PLATFORM
void app_main(void)
{
    app_run();
}
#else
void hal_host_args(int argc, char **argv);

int main(int argc, char **argv)
{
    hal_host_args(argc, argv);
    app_run();
    return 0;
}
#endif
