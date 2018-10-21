/*
    pa_mono_tas5760m.c - firmware for a single-channel power-amplifier module based on a TAS5760M
    class-D amplifier.

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "pa_mono_tas5760m.h"
#include "core/control.h"
#include "dev/lc89091ja.h"
#include "dev/pca9632.h"
#include "dev/sc16is752.h"
#include "dev/tas5760m.h"
#include "lib/adc.h"
#include "lib/debug.h"
#include "lib/twi.h"
#include "lib/vref.h"
#include "platform.h"
#include "util/irq.h"
#include <avr/io.h>
#include <util/delay.h>


#ifdef MODULE_PA_MONO_TAS5760M

#define AMP_VCC_STARTUP_THRESHOLD       (20000)     // Voltage above which to start the amplifier
#define AMP_INIT_RETRY_DELAY_MS         (100)       // Amp initialisation retry interval in ms

// PATAS5760MSleepMode_t - enumeration mapping control API sleep modes to sleep modes supported by
// this module.
typedef enum PATAS5760MSleepMode
{
    PASMShutdown    = ArgPowerStateMinimum,     // Lowest-power mode
    PASMSleep       = ArgPowerStateSleep,       // Sleep mode
    PASMNormal      = ArgPowerStateFull         // Active mode
} PATAS5760MSleepMode_t;

static uint8_t set_sleep_mode(const PATAS5760MSleepMode_t mode);
static uint16_t amp_get_vcc();


// ISR for pin-change events on GPIO port C.  This function is called when a change-of-state occurs
// on the nSPK_FAULT input, or when the SC16IS752 dual UART asserts its nIRQ output.
//
ISR(PORTC_PORT_vect)
{
    if(PORTC_INTFLAGS & gpio_pin_bit(PIN_nSPK_FAULT))
        tas5760m_isr_fault();

    if(PORTC_INTFLAGS & gpio_pin_bit(PIN_nUART_IRQ))
        ctrl_serial_isr();

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

    interrupt_enable_increment();

    lc89091ja_init();                       // Initialise the LC89091 digital receiver
    tas5760m_interface_init();              // Initialise the interface with the TAS5760M
    pca9632_init();                         // Initialise the PCA9632 LED driver
    ctrl_init();                            // Initialise the control interface

    // Set LED to red during initialisation
    pca9632_sleep(0);
    pca9632_pwm_set_all(0xff, 0x00, 0x00, 0x00);

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
    // returns success.  FIXME: give up and report an error condition to the controller.
    while(!tas5760m_init())
    {
        debug_putstr_p("TAS5760M: init failed\n");
        _delay_ms(AMP_INIT_RETRY_DELAY_MS);
    }

    tas5760m_set_gain(-3);
    tas5760m_mute(0);           // Unmute the amplifier

    // Set LED to green to indicate normal operations
    set_sleep_mode(PASMNormal);

    // Worker loop
    while(1)
    {
        tas5760m_worker();
        ctrl_worker();
    }
}


// set_sleep_mode() - configure the module's power state according to the sleep mode specified by
// <mode>.  Returns non-zero on success, zero on failure.
//
static uint8_t set_sleep_mode(const PATAS5760MSleepMode_t mode)
{
    // For "sleep" and "shutdown" modes, start by putting the amplifier to sleep; for all other
    // modes, wake the amplifier.
    if(!tas5760m_sleep((mode == PASMSleep) || (mode == PASMShutdown) ? 1 : 0))
        return 0;

    if(mode == PASMSleep)
    {
        // For sleep mode, put the amplifier IC to sleep, wake the LED driver IC, and make the LED
        // blink turquoise intermittently.
        if(!tas5760m_sleep(1))
            return 0;

        pca9632_sleep(0);
        pca9632_mode_set(PCA9632ModeBlink);             // Enter blink mode
        pca9632_pwm_set_all(0x00, 0x1f, 0x08, 0x00);    // Sleep colour = dim turquoise
        pca9632_group_freq_set(119);                    // Blink LED every 5s (=(119+1)/24)
        pca9632_group_pwm_set(3);                       // Set 1% duty cycle (=3/256) for blinking

    }
    else if(mode == PASMShutdown)
    {
        // For shutdown mode: shut down the amplifier IC, try to switch off the status LED, and
        // put the LED driver IC to sleep.  If the LED updates don't work, it's not really a
        // problem (hence errors are ignored).
        if(!tas5760m_sleep(1))
            return 0;

        pca9632_pwm_set_all(0x00, 0x00, 0x00, 0x00);
        pca9632_sleep(1);

        // For "shutdown" mode, put the amplifier in its lowest-power state.
        if(!tas5760m_shut_down(1))
            return 0;
    }
    else        // Assume all other cases represent "normal" (=fully active) mode
    {
        if(!tas5760m_shut_down(0))
            return 0;

        pca9632_sleep(0);
        pca9632_mode_set(PCA9632ModeDim);               // Enter dim mode
        pca9632_pwm_set_all(0x00, 0xff, 0x00, 0x00);    // Awake colour = green
        pca9632_group_pwm_set(0x20);                    // Set 1/8th brightness

        if(!tas5760m_sleep(0))
            return 0;
    }

    return 1;
}


//
// Handlers for control commands
//


// ctrl_identify() - provide device identity
//
CtrlResponse_t ctrl_identify(const uint8_t block)
{
    return CtrlRespOperationFailed;     // FIXME
}


// ctrl_set_power_state() - set the power state (sleep, wake, etc.) of the peripheral
//
CtrlResponse_t ctrl_set_power_state(const CtrlArgPowerState_t state)
{
    return set_sleep_mode(state) ? CtrlRespOK : CtrlRespOperationFailed;
}


// ctrl_set_channel() - set the channel (e.g. left or right) to be amplified by this module.
//
CtrlResponse_t ctrl_set_channel(const CtrlArgChannel_t channel)
{
    TAS5760MPBTLChannel_t ch;

    if(channel == CtrlAmpChannelLeft)
        ch = TAS5760MPBTLChannelLeft;
    else if(channel == CtrlAmpChannelRight)
        ch = TAS5760MPBTLChannelRight;
    else
        return CtrlRespBadArg;

    return tas5760m_set_pbtl_channel(ch) ? CtrlRespOK : CtrlRespOperationFailed;
}


// ctrl_set_gain() - set the gain of the amplifier to the value specified by <gain>, which uses
// units of 0.5dB.
//
CtrlResponse_t ctrl_set_gain(const int8_t gain)
{
    return tas5760m_set_gain(gain) ? CtrlRespOK : CtrlRespOperationFailed;
}


// ctrl_set_pairing() - set the (e.g. Bluetooth) pairing state of the peripheral.  Not supported by
// this device.
//
CtrlResponse_t ctrl_set_pairing(const CtrlArgPairingState_t state)
{
    return CtrlRespUnsupportedOperation;
}

#endif // MODULE_PA_MONO_TAS5760M
