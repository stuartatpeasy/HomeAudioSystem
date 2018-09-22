/*
    main.c - entry point

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "lib/clk.h"
#include "lib/debug.h"
#include "modules/pa_mono_tas5760m.h"


// Entry point
//
int main(void)
{
    pclk_set_divisor_val(2);                // Set peripheral clock = main clock / 2
    pclk_enable();                          // Enable peripheral clock

    debug_init();
    debug_putstr_p("Home audio system firmware\n");

    firmware_main();
}
