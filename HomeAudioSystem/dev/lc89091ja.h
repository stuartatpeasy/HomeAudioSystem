#ifndef DEV_LC89091JA_H_INC
#define DEV_LC89091JA_H_INC
/*
    lc89091ja.h  - declarations related to the driver for the LC89091JA-H digital audio interface
    receiver.

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "platform.h"
#include <stdint.h>

#ifdef MODULE_PA_MONO_TAS5760M


void lc89091ja_init();
void lc89091ja_worker();
uint8_t lc89091_get_error_state();

#endif // MODULE_PA_MONO_TAS5760M
#endif
