#ifndef LIB_CLK_H_INC
#define LIB_CLK_H_INC
/*
    clk.h: declarations of helper functions relating to clock control

    Stuart Wallace <stuartw@atom.net>, July 2018.
*/

#include "../platform.h"        // for F_CPU
#include <avr/io.h>


#define PCLK_DIVISOR_RESERVED       ((CLKCTRL_PDIV_t) 0xff)

#define pclk_is_enabled()           (CLKCTRL_MCLKCTRLB & CLKCTRL_PEN_bm)

void pclk_set_divisor(const CLKCTRL_PDIV_t divisor);
uint8_t pclk_set_divisor_val(const uint8_t val);
uint8_t pclk_get_divisor_val();
void pclk_enable();
void pclk_disable();
uint32_t pclk_get_freq();

#endif
