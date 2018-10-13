/*
    adc.c: definitions relating to the uC's ADC module

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "adc.h"


// adc_set_vref() - specify the source of the voltage reference for the ADC module.  Also specify
// whether the internal sampling capacitor value should be reduced (this is recommended if the
// reference voltage is above 1V).
//
void adc_set_vref(const ADCRef_t ref, const uint8_t reduce_sample_cap)
{
    uint8_t regval = (ADC0_CTRLC & ~ADC_REFSEL_gm) | (ref << ADC_REFSEL_gp);

    if(reduce_sample_cap)
        regval |= ADC_SAMPCAP_bm;
    else
        regval &= ~ADC_SAMPCAP_bm;

    ADC0_CTRLC = regval;
}


// adc_set_prescaler() - set the ADC clock prescaler division ratio.
//
void adc_set_prescaler(const ADCPrescaleDiv_t div)
{
    ADC0_CTRLC = (ADC0_CTRLC & ~ADC_PRESC_gm) | (div << ADC_PRESC_gp);
}


// adc_set_initdelay() - set the delay, expressed as a number of ADC clock cycles, following ADC
// startup (or reference change) before the first conversion can occur.
//
void adc_set_initdelay(const ADCInitDelay_t delay)
{
    ADC0_CTRLD = (ADC0_CTRLD & ~ADC_INITDLY_gm) | (delay << ADC_INITDLY_gp);
}


// adc_configure_input() - prepare the specified pin to act as an ADC input by disabling its
// digital input buffer and making it an input.
//
void adc_configure_input(const GPIOPin_t pin)
{
    gpio_set_sense(pin, GPIOSenseInputDisable);
    gpio_make_input(pin);
}


// adc_enable() - enable (if <enable> is non-zero) or disable (if <enable> equals zero) the ADC.
//
void adc_enable(const uint8_t enable)
{
    const uint8_t mask = ADC_ENABLE_bm << ADC_ENABLE_bp;

    if(enable)
        ADC0_CTRLA |= mask;
    else
        ADC0_CTRLA &= ~mask;
}


// adc_set_channel() - connect the channel specified by <channel> to the ADC input.
//
void adc_set_channel(const ADCChannel_t channel)
{
    ADC0_MUXPOS = channel;
}


// adc_convert() - perform a conversion on the currently-selected channel and return the result.
//
uint16_t adc_convert()
{
    ADC0_COMMAND |= ADC_STCONV_bm;              // Start conversion

    while(!(ADC0_INTFLAGS & ADC_RESRDY_bm))     // Wait for conversion to complete
        ;

    return ADC0_RES;
}


// adc_convert_channel() - connect the channel specified by <channel> to the ADC input, perform a
// conversion, and return the result.
//
uint16_t adc_convert_channel(const ADCChannel_t channel)
{
    adc_set_channel(channel);
    return adc_convert();
}


// adc_channel_from_gpio() - given GPIO pin <pin>, return the ADC channel associated with the pin.
// If the pin does not represent an ADC channel, return ADCChannelGND.
//
ADCChannel_t adc_channel_from_gpio(const GPIOPin_t pin)
{
    const GPIOPort_t port = gpio_port(pin);
    const uint8_t p = gpio_pin(pin);

    // Port A implements ADC channels 0-7 on pins A0-A7
    if(port == GPIOPortA)
        return (ADCChannel_t) p;

    // Port B implements ADC channels 8-11 on pins B5, B4, B1 and B0
    if(port == GPIOPortB)
    {
        switch(p)
        {
            case 5:     return ADCChannel8;
            case 4:     return ADCChannel9;
            case 1:     return ADCChannel10;
            case 0:     return ADCChannel11;
        }
    }

    return ADCChannelGND;       // <pin> does not specify an ADC input pin
}
