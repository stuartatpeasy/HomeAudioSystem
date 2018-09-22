/*
    tas5760m.c - definitions relating to the driver for the TAS5760M I2S-input class-D audio
    amplifier IC.

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "tas5760m.h"
#include "../platform.h"
#include "../lib/debug.h"
#include "../lib/gpio.h"
#include "../lib/twi.h"
#include <util/delay.h>


#define TAS5760M_I2C_ADDR       (0x6c)      // Default I2C address of a TAS5760M amplifier
#define TAS5760M_DEVICE_ID      (0x00)      // ID from the "Device Identification" register

// Convert a volume-control input, expressed in dB, to a value suitable for the TAS5760M volume-
// control registers.
//
#define TAS5760M_DB_TO_VOL(dB)      (((dB) > 24) ? 255 : ((dB) < -100) ? 0 :(((dB) * 2) + 207))

// TAS5760MRegister_t - enumeration of the configuration registers in the TAS5760M amplifier IC.
//
typedef enum TAS5760MRegister
{
    TAS5760MRegDeviceID         = 0,        // Device identification
    TAS5760MRegPowerCtrl        = 1,        // Power control (and part of clipping level)
    TAS5760MRegDCtrl            = 2,        // Digital control
    TAS5760MRegVolCtrlCfg       = 3,
    TAS5760MRegLeftChVolCtrl    = 4,
    TAS5760MRegRightChVolCtrl   = 5,
    TAS5760MRegACtrl            = 6,
    TAS5760MRegFaultErr         = 8,
    TAS5760MRegDigitalClipper2  = 16,
    TAS5760MRegDigitalClipper1  = 17,
    TAS5760MRegEnd                      // Not a real register - end-of-list sentinel
} TAS5760MRegister_t;

//
// Power control register (0x01) bits
//
#define TAS5760M_PCTRL_SPK_SLEEP        (1 << 1)    // Sleep mode
#define TAS5760M_PCTRL_nSPK_SD          (1 << 0)    // Speaker shutdown

//
// Digital control register (0x02)
//
#define TAS5760M_DCTRL_HPF_BYPASS       (1 << 7)    // Bypass internal high-pass filter
#define TAS5760M_DCTRL_DS               (1 << 3)    // Enable double-speed mode

// TAS5760MDCtrlBoost_t - enumeration of available digital boost levels
typedef enum TAS5760MDCtrlBoost
{
    TAS5760MDCtrlBoost0dB       = (0 << 3),         // Digital signal is boosted by 0dB
    TAS5760MDCtrlBoost6dB       = (1 << 3),         // Digital signal is boosted by 6dB (default)
    TAS5760MDCtrlBoost12dB      = (2 << 3),         // Digital signal is boosted by 12dB
    TAS5760MDCtrlBoost18dB      = (3 << 3)          // Digital signal is boosted by 18dB
} TAS5760MDCtrlBoost_t;

// TAS5760MDCtrlFormat_t - enumeration of available serial audio input formats
typedef enum TAS5760MDCtrlFormat
{
    TAS5760MDCtrlFormat24bRJ    = (0 << 0),         // 24 bit, right-justified
    TAS5760MDCtrlFormat20bRJ    = (1 << 0),         // 20 bit, right-justified
    TAS5760MDCtrlFormat18bRJ    = (2 << 0),         // 18 bit, right-justified
    TAS5760MDCtrlFormat16bRJ    = (3 << 0),         // 16 bit, right-justified
    TAS5760MDCtrlFormatI2S      = (4 << 0),         // I2S (default)
    TAS5760MDCtrlFormatLJ       = (5 << 0)          // 16-24 bits, left-justified
} TAS5760MDCtrlFormat_t;

//
// Volume control configuration register (0x03)
//
#define TAS5760M_VOLCTRL_FADE           (1 << 7)    // Enable volume fade
#define TAS5760M_VOLCTRL_MUTE_R         (1 << 1)    // Mute right channel
#define TAS5760M_VOLCTRL_MUTE_L         (1 << 0)    // Mute left channel

//
// Analog control register (0x06)
//
#define TAS5760M_ACTRL_PBTL_ENABLE      (1 << 7)    // Enable parallel bridge-tied load mode
#define TAS5760M_ACTRL_PBTL_RIGHT       (0 << 1)    // Use right channel for PBTL
#define TAS5760M_ACTRL_PBTL_LEFT        (1 << 1)    // Use left channel for PBTL
#define TAS5760M_ACTRL_RESERVED_BIT0    (1 << 0)    // Bit 0 is reserved, and must be set to 1

// TAS5760MACtrlPWMRate_t - enumeration of available amplifier-output PWM switching ratios
typedef enum TAS5760MACtrlPWMRate
{
    TAS5760MACtrlPWMRate6       = (0 << 4),         // Switching rate is 6 * LRCLK
    TAS5760MACtrlPWMRate8       = (1 << 4),         // Switching rate is 8 * LRCLK
    TAS5760MACtrlPWMRate10      = (2 << 4),         // Switching rate is 10 * LRCLK
    TAS5760MACtrlPWMRate12      = (3 << 4),         // Switching rate is 12 * LRCLK
    TAS5760MACtrlPWMRate14      = (4 << 4),         // Switching rate is 14 * LRCLK
    TAS5760MACtrlPWMRate16      = (5 << 4),         // Switching rate is 16 * LRCLK (default)
    TAS5760MACtrlPWMRate20      = (6 << 4),         // Switching rate is 20 * LRCLK
    TAS5760MACtrlPWMRate24      = (7 << 4),         // Switching rate is 24 * LRCLK
} TAS5760MACtrlPWMRate_t;

// TAS5760MACtrlAGain - enumeration of available analogue-gain values.
typedef enum TAS5760MACtrlAGain
{
    TAS5760MACtrlAGain19_2dB    = (0 << 2),         // Analogue gain = 19.2dB
    TAS5760MACtrlAGain22_6dB    = (1 << 2),         // Analogue gain = 22.6dB
    TAS5760MACtrlAGain25dB      = (1 << 2),         // Analogue gain = 25dB
} TAS5760MACtrlAGain;

// Constants used in the "status" static variable.  These are updated by ISRs.
#define TAS5760M_STATUS_FAULT   (0x01)              // The amplifier has asserted nSPK_FAULT

static uint8_t sync_register_read(const TAS5760MRegister_t reg, uint8_t * const val);
static uint8_t sync_register_write(const TAS5760MRegister_t reg, const uint8_t val);

static volatile uint8_t status = 0;


// tas5760m_interface_init() - initialise the interface with the TAS5760M by configuring the
// uC SPK_FAULT pin as an input, configuring the uC nSPK_SD (shutdown) pin as an output, and
// asserting the nSPK_SD line to hold the amplifier in shutdown.
//
void tas5760m_interface_init()
{
    debug_putstr_p("TAS5760M: init interface\n");
    gpio_clear(PIN_nSPK_SD);

    gpio_make_input(PIN_nSPK_FAULT);
    gpio_make_output(PIN_nSPK_SD);

    gpio_set_sense(PIN_nSPK_FAULT, GPIOSenseBothEdges);
}


// tas5760m_init() - initialise the TAS5760M driver and attempt to detect the device.
//
uint8_t tas5760m_init()
{
    uint8_t id = 0;

    debug_putstr_p("TAS5760M: init\n");

    // Attempt to read the device ID.  Fail if the read fails, or the wrong ID is returned.
    if(!sync_register_read(TAS5760MRegDeviceID, &id) || (id != TAS5760M_DEVICE_ID))
        return 0;

    // Analogue configuration: enable PBTL, set PWM rate = 16 * LRCLK, set gain = 19.2dB, select
    // right channel for PBTL.
    if(!sync_register_write(TAS5760MRegACtrl, TAS5760M_ACTRL_PBTL_ENABLE | TAS5760MACtrlPWMRate16 |
                            TAS5760MACtrlAGain19_2dB | TAS5760M_ACTRL_PBTL_RIGHT |
                            TAS5760M_ACTRL_RESERVED_BIT0))
        return 0;

    // Digital configuration: set single speed, HPF not bypassed, I2S input format and 0dB digital
    // boost.
    if(!sync_register_write(TAS5760MRegDCtrl, TAS5760MDCtrlBoost0dB | TAS5760MDCtrlFormatI2S))
        return 0;

    // Mute left and right channels and enable fading.
    if(!sync_register_write(TAS5760MRegVolCtrlCfg, TAS5760M_VOLCTRL_FADE |
                            TAS5760M_VOLCTRL_MUTE_L | TAS5760M_VOLCTRL_MUTE_R))
        return 0;

    // Set volume control to 0dB for the left and right channels.
    if(!sync_register_write(TAS5760MRegLeftChVolCtrl, TAS5760M_DB_TO_VOL(0)) ||
       !sync_register_write(TAS5760MRegRightChVolCtrl, TAS5760M_DB_TO_VOL(0)))
        return 0;

    // Set digital clip level to maximum (0xfffff) and wake speaker
    if(!sync_register_write(TAS5760MRegDigitalClipper1, 0xfc) ||
       !sync_register_write(TAS5760MRegDigitalClipper2, 0xff) ||
       !sync_register_write(TAS5760MRegPowerCtrl, 0xfc | TAS5760M_PCTRL_nSPK_SD))
        return 0;

    gpio_set(PIN_nSPK_SD);      // Bring the device out of shutdown
    _delay_ms(10);              // Wait for the device to come out of shutdown

    return 1;
}


// tas5760m_sleep() - put the TAS5760M amplifier to sleep (if <sleep> is non-zero), or wake it up
// (if <sleep> equals zero) by asserting or negating the SPK_SLEEP pin.  This should be done around
// power-state changes in order to avoid audible artifacts associated with startup/shutdown.
//
uint8_t tas5760m_sleep(const uint8_t sleep)
{
    uint8_t pc_val = 0;

    if(!sync_register_read(TAS5760MRegPowerCtrl, &pc_val))
        return 0;

    // If the requested sleep state differs from the current sleep state, attempt to update the
    // state in the device.
    if(!(pc_val & TAS5760M_PCTRL_SPK_SLEEP) ^ !sleep)
    {
        if(sleep)
            pc_val |= TAS5760M_PCTRL_SPK_SLEEP;
        else
            pc_val &= ~TAS5760M_PCTRL_SPK_SLEEP;

        return sync_register_write(TAS5760MRegPowerCtrl, pc_val);
    }

    return 1;
}


// tas5760m_shut_down() - shut down (if <shut_down> is non-zero), or switch on (if <shut_down>
// equals zero) the TAS5760M amplifier.  Shutting the amplifier down reduces its power consumption
// to the lowest possible state.  Consider using tas5760m_sleep() around calls to this function in
// order to eliminate any audible artifacts arising from the change of power state.
//
uint8_t tas5760m_shut_down(const uint8_t shut_down)
{
    uint8_t pc_val = 0;

    if(!sync_register_read(TAS5760MRegPowerCtrl, &pc_val))
        return 0;

    if(!(pc_val & TAS5760M_PCTRL_nSPK_SD) ^ !shut_down)
        return 1;       // Nothing to do

    if(shut_down)
        pc_val &= ~TAS5760M_PCTRL_nSPK_SD;
    else
        pc_val |= TAS5760M_PCTRL_nSPK_SD;

    return sync_register_write(TAS5760MRegPowerCtrl, pc_val);
}


// sync_register_read() - convenience wrapper around twi_sync_register_read().  Avoids the need to
// specify TAS5760M_I2C_ADDR in every call.
//
static uint8_t sync_register_read(const TAS5760MRegister_t reg, uint8_t * const val)
{
    return twi_sync_register_read(TAS5760M_I2C_ADDR, reg, val) == TWICmdSuccess;
}


// sync_register_write() - convenience wrapper around twi_sync_register_write().  Avoids the need
// to specify TAS5760M_I2C_ADDR in every call.
//
static uint8_t sync_register_write(const TAS5760MRegister_t reg, const uint8_t val)
{
    return twi_sync_register_write(TAS5760M_I2C_ADDR, reg, val) == TWICmdSuccess;
}


// tas5760m_mute() - mute (if <mute> is non-zero) or un-mute (if <mute> equals zero) both channels
// of the amplifier.
//
uint8_t tas5760m_mute(const uint8_t mute)
{
    uint8_t vc_val = 0;

    if(!sync_register_read(TAS5760MRegVolCtrlCfg, &vc_val))
        return 0;

    if(mute)
        vc_val |= TAS5760M_VOLCTRL_MUTE_L | TAS5760M_VOLCTRL_MUTE_R;
    else
        vc_val &= ~(TAS5760M_VOLCTRL_MUTE_L | TAS5760M_VOLCTRL_MUTE_R);

    return sync_register_write(TAS5760MRegVolCtrlCfg, vc_val);
}


// tas5760m_isr_fault() - ISR associated with state changes on the GPIO pin connected to the
// amplifier's nSPK_FAULT output.
//
void tas5760m_isr_fault()
{
    if(!gpio_read(PIN_nSPK_FAULT))
    {
        // The amplifier has entered a fault condition.  Record this status and assert the
        // amplifier's hardware shutdown (nSPK_SD) line.
        gpio_clear(PIN_nSPK_SD);
        status |= TAS5760M_STATUS_FAULT;
    }
    else
    {
        // The amplifier is no longer in a fault state.  Clear the "fault" bit in the status byte
        // and bring the amplifier out of hardware shutdown.
        gpio_set(PIN_nSPK_SD);
        status &= ~TAS5760M_STATUS_FAULT;
    }
}


// tas5760m_worker() - worker function, to be called regularly.
//
void tas5760m_worker()
{
    if(status & TAS5760M_STATUS_FAULT)
    {
        uint8_t fault = 0;
        if(sync_register_read(TAS5760MRegFaultErr, &fault))
        {
            if(fault)
            {
                tas5760m_dump_fault(fault);
                if(fault & TAS5760M_FAULT_CLKE)
                    gpio_set(PIN_nSPK_SD);
            }
        }
        else
        {
            // Failed to read the fault/error register.
            // FIXME: shut down the amplifier
            debug_putstr_p("TAS5760M: failed to read fault reg\n");
        }
    }
}


#ifdef DEBUG

// tas5760m_dump_registers() - if a debug build is running, dump the contents of the TAS5760M
// internal registers to the debug console.
//
void tas5760m_dump_registers()
{
    TAS5760MRegister_t r;
    uint8_t val = 0;

    debug_putstr_p("\n==== TAS5760M register dump ====\n");
    for(r = 0; r < TAS5760MRegEnd; ++r)
    {
        if(sync_register_read(r, &val))
            debug_printf("R%02d = %02x\n", r, val);
        else
            debug_putstr_p("(register read failed)\n");
    }
}


// tas5760m_dump_fault() - in a debug build, write a human-readable description of the contents of
// the TAS5760M fault/error register (as specified by <fault>) to the debug console.
//
void tas5760m_dump_fault(const uint8_t fault)
{
    debug_putstr_p("TAS5760M fault status: ");

    if(!fault)
        debug_putstr_p("no fault");

    if(fault & TAS5760M_FAULT_CLKE)
        debug_putstr_p("clk_error ");
    if(fault & TAS5760M_FAULT_OCE)
        debug_putstr_p("overcurrent_error ");
    if(fault & TAS5760M_FAULT_DCE)
        debug_putstr_p("dc_offset_error ");
    if(fault & TAS5760M_FAULT_OTE)
        debug_putstr_p("overtemp_error ");

    debug_putstr_p("\n");
}

#endif
