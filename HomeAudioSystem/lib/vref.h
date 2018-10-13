#ifndef LIB_VREF_H_INC
#define LIB_VREF_H_INC
/*
    vref.h: declarations relating to the uC's voltage reference (VRef) module

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include <stdint.h>


// VRefPeripheral_t - enumeration specifying the peripherals connected to the VRef module.
//
typedef enum VRefPeripheral
{
    VRefADC0,           // Specifies the voltage reference output connected to ADC0
    VRefDAC0            // Specifies the voltage reference output connected to DAC0
} VRefPeripheral_t;


// VRefVoltage_t - enumeration of the various voltages supported by the VRef peripheral.
//
typedef enum VRefVoltage
{
    VRef0V55    = 0,    // Specifies an internal voltage reference of 0.55V
    VRef1V1     = 1,    // Specifies an internal voltage reference of 1.1V
    VRef2V5     = 2,    // Specifies an internal voltage reference of 2.5V
    VRef4V3     = 3,    // Specifies an internal voltage reference of 4.3V
    VRef1V5     = 4     // Specifies an internal voltage reference of 1.5V
} VRefVoltage_t;


void vref_set(const VRefPeripheral_t peripheral, const VRefVoltage_t voltage);
void vref_enable(const VRefPeripheral_t peripheral, const uint8_t enable);

#endif
