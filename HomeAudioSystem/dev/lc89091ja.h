#ifndef DEV_LC89091JA_H_INC
#define DEV_LC89091JA_H_INC
/*
    lc89091ja.h  - declarations related to the driver for the LC89091JA-H digital audio interface
    receiver.

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "platform.h"
#include <stdint.h>

#ifdef WITH_LC89091JA


void lc89091ja_init();
uint8_t lc89091_get_error_state();

#endif // WITH_LC89091JA
#endif // DEV_LC89091JA_H_INC
