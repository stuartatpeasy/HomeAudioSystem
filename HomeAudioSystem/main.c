/*
    main.c - entry point

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "platform.h"       // for MODULE_TYPE
#include "lib/adc.h"
#include "lib/clk.h"
#include "lib/debug.h"
#include "lib/vref.h"

#if defined(MODULE_PA_MONO_TAS5760M)
#include "modules/pa_mono_tas5760m.h"
#elif defined(MODULE_BT_SOURCE_V1)
#include "modules/bt_source_v1.h"
#else
#error "No module type specified - check platform.h"
#endif

int8_t g_irq_state = 0;     // Used in util/irq.h


#if defined(PIN_VCC_IN)

// sys_get_vcc() - read the ADC channel connected to a scaled representation of the system supply
// voltage, and return the voltage, in millivolts, as a uint32_t.
//
static uint32_t sys_get_vcc()
{
    // The system supply voltage passes through a voltage-divider consisting of a 2.4k and a 91k
    // resistor before entering the uC's ADC.  The ADC's reference voltage is 2.5V, and this
    // corresponds to an ADC conversion reading of 1023.  The system Vcc is therefore given by:
    //
    //      Vcc = (93.4 / 2.4) * 2.5 * (adc_reading / 1023)
    //          = 0.0951 * adc_reading
    //
    // For a Vcc reading in millivolts, this becomes approximately:
    //
    //      Vcc(mV) = 95 * adc_reading

    adc_set_channel(adc_channel_from_gpio(PIN_VCC_IN));
    return 95 * adc_convert();
}

#endif // PIN_VCC_IN


// Entry point
//
int main(void)
{
    pclk_set_divisor_val(2);                // Set peripheral clock = main clock / 2
    pclk_enable();                          // Enable peripheral clock

    debug_init();
    debug_putstr_p("Home audio system firmware\n");

    vref_set(VRefADC0, VRef2V5);
    vref_enable(VRefADC0, 1);

    adc_set_vref(ADCRefInternal, 1);
    adc_set_prescaler(ADCPrescaleDiv16);
    adc_set_initdelay(ADCInitDelay16);
    adc_enable(1);

#if defined(PIN_VCC_IN)
    adc_configure_input(PIN_VCC_IN);
    debug_printf("Vcc(in) = %lumV\n", sys_get_vcc());
#endif // PIN_VCC_IN

    // firmware_main() should loop eternally; loop here in case it doesn't.
    while(1)
        firmware_main();        // should never return
}
