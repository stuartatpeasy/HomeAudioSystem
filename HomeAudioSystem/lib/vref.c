/*
    vref.c: declarations relating to the uC's voltage reference module

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "vref.h"
#include <avr/io.h>


// vref_set() - set the reference voltage for the specified peripheral.
//
void vref_set(const VRefPeripheral_t peripheral, const VRefVoltage_t voltage)
{
    uint8_t offset = 0, mask = 0;

    switch(peripheral)
    {
        case VRefADC0:
            offset = VREF_ADC0REFSEL_gp;
            mask = VREF_ADC0REFSEL_gm;
            break;

        case VRefDAC0:
            offset = VREF_DAC0REFSEL_gp;
            mask = VREF_DAC0REFSEL_gm;
            break;
    }

    VREF_CTRLA &= ~mask;
    VREF_CTRLA |= voltage << offset;
}


// vref_enable() - enable or disable the voltage reference for the specified peripheral.
//
void vref_enable(const VRefPeripheral_t peripheral, const uint8_t enable)
{
    uint8_t bit = 8;    // Invalid bit number by default, so that setting/clearing has no effect.

    switch(peripheral)
    {
        case VRefADC0:
            bit = 0;
            break;

        case VRefDAC0:
            bit = 1;
            break;
    }

    if(enable)
        VREF_CTRLB |= 1 << bit;
    else
        VREF_CTRLB &= ~(1 << bit);
}
