/*
    clk.c: declarations of helper functions relating to clock control

    Stuart Wallace <stuartw@atom.net>, July 2018.
 */

#include "clk.h"
#include <avr/pgmspace.h>


// Map of peripheral clock (PCLK) divisor values, as stored in bits 1:4 of MCLKCTRLB, to actual
// divisor values.
static const uint8_t pclk_divisor_to_uint_map[16] PROGMEM =
{
    2, 4, 8, 16, 32, 64, PCLK_DIVISOR_RESERVED, PCLK_DIVISOR_RESERVED,
    6, 10, 12, 24, 48, PCLK_DIVISOR_RESERVED, PCLK_DIVISOR_RESERVED, PCLK_DIVISOR_RESERVED
};


// pclk_enable() - enable the peripheral clock.  Interrupts must be disabled around this call.
//
void pclk_enable()
{
    const uint8_t mclkctrlb_val = CLKCTRL_MCLKCTRLB | CLKCTRL_PEN_bm;

    CPU_CCP = CCP_IOREG_gc;                 // Unlock config change protection for IO regs
    CLKCTRL_MCLKCTRLB = mclkctrlb_val;      // Enable the peripheral clock.
}


// pclk_disable() - disable the peripheral clock.  Interrupts must be disabled around this call.
//
void pclk_disable()
{
    const uint8_t mclkctrlb_val = CLKCTRL_MCLKCTRLB & ~CLKCTRL_PEN_bm;

    CPU_CCP = CCP_IOREG_gc;                 // Unlock config change protection for IO regs
    CLKCTRL_MCLKCTRLB = mclkctrlb_val;      // Enable the peripheral clock.
}


// pclk_set_divisor() - Set the peripheral clock (pclk) divisor to the specified value.  Interrupts must be disabled around this call.
//
void pclk_set_divisor(const CLKCTRL_PDIV_t divisor)
{
    // Calculate new value for MCLKCTRLB register, preserving all bits except those holding the
    // PCLK divisor (PDIV) value.
    const uint8_t mclkctrlb_val = (CLKCTRL_MCLKCTRLB & ~CLKCTRL_PDIV_gm) | divisor;

    CPU_CCP = CCP_IOREG_gc;                 // Unlock config change protection for IO regs
    CLKCTRL_MCLKCTRLB = mclkctrlb_val;      // Set new PCLK divisor value
}


// pclk_set_divisor_val() - set the peripheral clock divisor to the value specified by <val> (a
// uint).  Return 1 on success, or 0 if <val> represents an invalid/unsupported divider value.
//
uint8_t pclk_set_divisor_val(const uint8_t val)
{
    CLKCTRL_PDIV_t divisor;

    switch(val)
    {
        case 2:     divisor = CLKCTRL_PDIV_2X_gc;       break;
        case 4:     divisor = CLKCTRL_PDIV_4X_gc;       break;
        case 6:     divisor = CLKCTRL_PDIV_6X_gc;       break;
        case 8:     divisor = CLKCTRL_PDIV_8X_gc;       break;
        case 10:    divisor = CLKCTRL_PDIV_10X_gc;      break;
        case 12:    divisor = CLKCTRL_PDIV_12X_gc;      break;
        case 16:    divisor = CLKCTRL_PDIV_16X_gc;      break;
        case 24:    divisor = CLKCTRL_PDIV_24X_gc;      break;
        case 32:    divisor = CLKCTRL_PDIV_32X_gc;      break;
        case 64:    divisor = CLKCTRL_PDIV_64X_gc;      break;
        default:    divisor = PCLK_DIVISOR_RESERVED;    break;
    }

    if(divisor == PCLK_DIVISOR_RESERVED)
        return 0;       // Invalid divisor

    pclk_set_divisor(divisor);
    return 1;
}


// pclk_get_divisor_val() - return the absolute value (i.e. an integer, not one of the enumerated
// constants from CLKCTRL_PDIV_t) of the peripheral clock (PCLK) divisor.
uint8_t pclk_get_divisor_val()
{
    const uint8_t pclk_divisor = (CLKCTRL_MCLKCTRLB & CLKCTRL_PDIV_gm) >> CLKCTRL_PDIV0_bp;

    return pgm_read_byte(pclk_divisor_to_uint_map + pclk_divisor);
}


// pclk_get_freq() - get the frequency of the peripheral clock, in Hz.  If the current peripheral
// clock divisor has an invalid/unsupported value, this function returns 0.  This function does not
// consider whether or not the peripheral clock is enabled.
//
uint32_t pclk_get_freq()
{
    const uint8_t divisor_val = pclk_get_divisor_val();

    return (divisor_val == PCLK_DIVISOR_RESERVED) ? 0 : F_CPU / divisor_val;
}
