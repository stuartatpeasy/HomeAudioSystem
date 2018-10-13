/*
    pa_mono_tas5760m.c - firmware for a single-channel power-amplifier module based on a TAS5760M
    class-D amplifier.

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "pa_mono_tas5760m.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include "../dev/lc89091ja.h"
#include "../dev/sc16is752.h"
#include "../dev/tas5760m.h"
#include "../lib/adc.h"
#include "../lib/debug.h"
#include "../lib/twi.h"
#include "../lib/vref.h"
#include "../platform.h"
#include <util./delay.h>


#ifdef MODULE_PA_MONO_TAS5760M

#define AMP_VCC_STARTUP_THRESHOLD       (20000)     // Voltage above which to start the amplifier
#define AMP_INIT_RETRY_DELAY_MS         (100)       // Amp initialisation retry interval in ms

static uint16_t amp_get_vcc();


// ISR for pin-change events on GPIO port A.  This function is called when a change-of-state occurs
// on the nSPK_FAULT input.
//
ISR(PORTC_PORT_vect)
{
    if(PORTC_INTFLAGS & gpio_pin_bit(PIN_nSPK_FAULT))
        tas5760m_isr_fault();

    if(PORTC_INTFLAGS & gpio_pin_bit(PIN_nUART_IRQ))
        sc16is752_isr();

    PORTC_INTFLAGS = 0xff;      // Clear all pin interrupts
}


// amp_get_vcc() - read the ADC channel connected to a scaled representation of the amplifier Vcc
// (supply voltage), and return the voltage, in millivolts, as a uint16_t.
//
static uint16_t amp_get_vcc()
{
    // The amplifier's Vcc passes through a voltage-divider consisting of a 2.7k and a 47k resistor
    // before entering the uC's ADC.  The ADC's reference voltage is 2.5V, and this corresponds to
    // an ADC conversion reading of 1023.  The amplifier Vcc is therefore given by:
    //
    //      Vcc = (49.7 / 2.7) * 2.5 * (adc_reading / 1023)
    //          = 0.04498 * adc_reading
    //
    // For a Vcc reading in millivolts, this becomes approximately:
    //
    //      Vcc(mV) = 45 * adc_reading

    adc_set_channel(adc_channel_from_gpio(PIN_VCC_AMP));
    return 45 * adc_convert();
}


// Firmware entry point
//
void firmware_main()
{
    uint16_t amp_vcc_mv;                    // Will hold the amplifier Vcc in millivolts

    debug_putstr_p("TAS5760M mono PA module\n\n");

    adc_configure_input(PIN_VCC_AMP);

    twi_configure_master(PinsetDefault);
    twi_master_enable(1);
    twi_set_clock(TWISpeed_400kHz);

    sei();

    sc16is752_init();                       // Initialise the SC16IS752 dual UART
    lc89091ja_init();                       // Initialise the LC89091 digital receiver
    tas5760m_interface_init();              // Initialise the interface with the TAS5760M

    for(SC16IS752Channel_t c = SC16IS752ChannelA; c <= SC16IS752ChannelB; ++c)
    {
        sc16is752_set_baud_rate(c, 230400);
        sc16is752_enable_auto_rts(c, 1, 1);
    }

    // Given that the amplifier module is powered via a PoE-style interface, i.e. is physically
    // remote from the power source, and the local dc-dc converters drive significant bulk
    // capacitance, it can take "a while" for the power supplies to stabilise.  Wait for the
    // amplifier's supply voltage to reach the activation threshold before attempting to initialise
    // the TAS5760M.
    do
    {
        _delay_ms(100);
        amp_vcc_mv = amp_get_vcc();
        debug_printf("Vcc(amp) = %umV\n", amp_vcc_mv);
    } while(amp_vcc_mv < AMP_VCC_STARTUP_THRESHOLD);

    // Attempt to initialise and configure the amplifier module by calling tas5760m_init() until it
    // returns success.  TODO: give up and report an error condition to the controller.
    while(!tas5760m_init())
    {
        debug_putstr_p("TAS5760M: init failed\n");
        _delay_ms(AMP_INIT_RETRY_DELAY_MS);
    }

    tas5760m_set_volume(-3);
    tas5760m_mute(0);           // Un-mute the amplifier


    // Worker loop
    while(1)
    {
        _delay_ms(100);
        lc89091ja_worker();
        tas5760m_worker();
    }
}

#endif // MODULE_PA_MONO_TAS5760M
