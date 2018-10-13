#ifndef LIB_ADC_H_INC
#define LIB_ADC_H_INC
/*
    adc.h: declarations relating to the uC's ADC module

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include <stdint.h>
#include <avr/io.h>
#include "gpio.h"


// ADCRef_t - enumeration of possible voltage reference sources.
//
typedef enum ADCRef
{
    ADCRefInternal      = 0,        // Use the internal VREF peripheral as a voltage reference
    ADCRefVDD           = 1         // Use VDD as a voltage reference
} ADCRef_t;


// ADCPrescaleDiv_t - enumeration of possible values of the ADC clock prescaler.
//
typedef enum ADCPrescaleDiv
{
    ADCPrescaleDiv2     = ADC_PRESC_DIV2_gc,        // ADC clock = PCLK/2
    ADCPrescaleDiv4     = ADC_PRESC_DIV4_gc,        // ADC clock = PCLK/4
    ADCPrescaleDiv8     = ADC_PRESC_DIV8_gc,        // ADC clock = PCLK/8
    ADCPrescaleDiv16    = ADC_PRESC_DIV16_gc,       // ADC clock = PCLK/16
    ADCPrescaleDiv32    = ADC_PRESC_DIV32_gc,       // ADC clock = PCLK/32
    ADCPrescaleDiv64    = ADC_PRESC_DIV64_gc,       // ADC clock = PCLK/64
    ADCPrescaleDiv128   = ADC_PRESC_DIV128_gc,      // ADC clock = PCLK/128
    ADCPrescaleDiv256   = ADC_PRESC_DIV256_gc       // ADC clock = PCLK/256
} ADCPrescaleDiv_t;


// ADCInitDelay_t - enumeration of possible values of the initialisation delay.  This delay is
// applied when the ADC starts up, or the reference changes, so that the ADC input may settle
// before the first conversion is performed.
//
typedef enum ADCInitDelay
{
    ADCInitDelay0       = ADC_INITDLY_DLY0_gc,      // Init delay = 0 ADC CLK cycles
    ADCInitDelay16      = ADC_INITDLY_DLY16_gc,     // Init delay = 16 ADC CLK cycles
    ADCInitDelay32      = ADC_INITDLY_DLY32_gc,     // Init delay = 32 ADC CLK cycles
    ADCInitDelay64      = ADC_INITDLY_DLY64_gc,     // Init delay = 64 ADC CLK cycles
    ADCInitDelay128     = ADC_INITDLY_DLY128_gc,    // Init delay = 128 ADC CLK cycles
    ADCInitDelay256     = ADC_INITDLY_DLY256_gc     // Init delay = 256 ADC CLK cycles
} ADCInitDelay_t;


// ADCChannel_t- enumeration of possible ADC input channel values.
//
typedef enum ADCChannel
{
    ADCChannel0         = ADC_MUXPOS_AIN0_gc,       // ADC channel 0
    ADCChannel1         = ADC_MUXPOS_AIN1_gc,       // ADC channel 1
    ADCChannel2         = ADC_MUXPOS_AIN2_gc,       // ADC channel 2
    ADCChannel3         = ADC_MUXPOS_AIN3_gc,       // ADC channel 3
    ADCChannel4         = ADC_MUXPOS_AIN4_gc,       // ADC channel 4
    ADCChannel5         = ADC_MUXPOS_AIN5_gc,       // ADC channel 5
    ADCChannel6         = ADC_MUXPOS_AIN6_gc,       // ADC channel 6
    ADCChannel7         = ADC_MUXPOS_AIN7_gc,       // ADC channel 7
    ADCChannel8         = ADC_MUXPOS_AIN8_gc,       // ADC channel 8
    ADCChannel9         = ADC_MUXPOS_AIN9_gc,       // ADC channel 9
    ADCChannel10        = ADC_MUXPOS_AIN10_gc,      // ADC channel 10
    ADCChannel11        = ADC_MUXPOS_AIN11_gc,      // ADC channel 11
    ADCChannelDAC0      = ADC_MUXPOS_DAC0_gc,       // Connect ADC to DAC output
    ADCChannelIntRef    = ADC_MUXPOS_INTREF_gc,     // Connect ADC to internal voltage ref
    ADCChannelTempSense = ADC_MUXPOS_TEMPSENSE_gc,  // Connect ADC to on-chip temp sensor
    ADCChannelGND       = ADC_MUXPOS_GND_gc         // Connect ADC to ground
} ADCChannel_t;


void adc_set_vref(const ADCRef_t ref, const uint8_t reduce_sample_cap);
void adc_set_prescaler(const ADCPrescaleDiv_t div);
void adc_set_initdelay(const ADCInitDelay_t delay);
void adc_enable(const uint8_t enable);
void adc_set_channel(const ADCChannel_t channel);
uint16_t adc_convert();
uint16_t adc_convert_channel(const ADCChannel_t channel);
void adc_configure_input(const GPIOPin_t pin);
ADCChannel_t adc_channel_from_gpio(const GPIOPin_t pin);

#endif
