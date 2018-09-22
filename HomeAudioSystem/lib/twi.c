/*
    twi.c - definitions relating to the two-wire interface (TWI) peripheral

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "twi.h"
#include "clk.h"
#include "debug.h"
#include <avr/interrupt.h>
#include <avr/io.h>


#define TWI_CMD_WRITE           (0)     // Bit value indicating that the master is writing
#define TWI_CMD_READ            (1)     // Bit value indicating that the master is reading


// TWIBusState_t - enumeration of the possible states of the TWI bus, as reported in the TWI
// MSTATUS register.
//
typedef enum TWIBusState
{
    TWIBusStateUnknown  = 0,        // Unknown bus state
    TWIBusStateIdle     = 1,        // Bus is idle
    TWIBusStateOwner    = 2,        // This TWI controls the bus
    TWIBusStateBusy     = 3         // The bus is busy
} TWIBusState_t;

static volatile struct TWICommand
{
    uint8_t write;              // 0 = read, 1 = write
    uint8_t dev_addr;           // TWI device address
    uint8_t reg_addr;           // TWI device register address
    uint8_t data;               // Data to be written, or data received from the device
    TWICmdState_t state;        // Current state of the FSM
} twi_command;


static TWIBusState_t twi_bus_status();
static uint8_t twi_cmd_start(const uint8_t dev_addr, const uint8_t reg_addr, const uint8_t data,
                             const uint8_t is_write);
static TWICmdStatus_t twi_sync_cmd(const uint8_t dev_addr, const uint8_t reg_addr,
                                   uint8_t * const data, const uint8_t is_write);


// ISR(TWI0_TWIM_vect) - ISR which handles master transmit and receive interrupt events on the TWI
// bus.
//
ISR(TWI0_TWIM_vect)
{
    if(TWI0_MSTATUS & TWI_WIF_bm)
    {
        if(TWI0_MSTATUS & TWI_RXACK_bm)
        {
            twi_command.state = TWICmdStateNack;            // NACK or no acknowledgment received

            // Issue a STOP to end the transaction
            TWI0_MCTRLB = (TWI0_MCTRLB & ~(TWI_MCMD_gm | TWI_ACKACT_bm)) | TWI_MCMD_STOP_gc;
            return;
        }

        switch(twi_command.state)
        {
            case TWICmdDevAddr:
                TWI0_MDATA = twi_command.reg_addr;          // Transmit register address
                twi_command.state = TWICmdRegAddr;
                break;

            case TWICmdRegAddr:
                if(twi_command.write)
                {
                    TWI0_MDATA = twi_command.data;          // Transmit data
                    twi_command.state = TWICmdDataWrite;
                }
                else
                {
                    // Transmit device address and request a read operation
                    TWI0_MADDR = (twi_command.dev_addr << 1) | TWI_CMD_READ;
                    twi_command.state = TWICmdDataRead;
                }
                break;

            case TWICmdDataRead:
                break;      // No action - case handled by the RIF handler below

            case TWICmdDataWrite:
                // Issue a STOP command
                TWI0_MCTRLB = (TWI0_MCTRLB & ~(TWI_MCMD_gm | TWI_ACKACT_bm)) | TWI_MCMD_STOP_gc;
                twi_command.state = TWICmdStateIdle;

                // Return the bus to the idle state
                TWI0_MSTATUS = (TWI0_MSTATUS & ~TWI_BUSSTATE_gm) | TWI_BUSSTATE_IDLE_gc;
                break;

            default:
                twi_command.state = TWICmdStateError;
                break;
        }
    }

    if(TWI0_MSTATUS & TWI_RIF_bm)
    {
        if(twi_command.state == TWICmdDataRead)
        {
            // Issue a NACK (to tell the slave that no more data is expected) and a STOP.
            TWI0_MCTRLB = (TWI0_MCTRLB & ~TWI_MCMD_gm) | TWI_MCMD_STOP_gc | TWI_ACKACT_bm;
            twi_command.data = TWI0_MDATA;

            // Return the bus to the idle state
            TWI0_MSTATUS = (TWI0_MSTATUS & ~TWI_BUSSTATE_gm) | TWI_BUSSTATE_IDLE_gc;

            twi_command.state = TWICmdStateIdle;
        }
        else
            twi_command.state = TWICmdStateError;
    }
}


// twi_configure_master() - configure the default or alternate pin-set for the TWI peripheral and
// enable "smart mode" (automatic ACK generation).
//
void twi_configure_master(const Pinset_t pinset)
{
    if(pinset == PinsetDefault)
        PORTMUX_CTRLB &= ~PORTMUX_TWI0_ALTERNATE_gc;
    else
        PORTMUX_CTRLB |= PORTMUX_TWI0_ALTERNATE_gc;

    // Enable "smart mode" (for automatic ACK generation)
    TWI0_MCTRLA |= TWI_SMEN_bm;

    // Force the bus to the "idle" state
    TWI0_MSTATUS = TWI_BUSSTATE_IDLE_gc;

    twi_command.state = TWICmdStateIdle;
}


// twi_master_enable() - enable (if <enable> is non-zero) or disable (if <enable> equals zero) the
// TWI master.  If this function enables the master, it also forces the bus to the "idle" state and
// enables the TWI transmit and receive interrupts.  The two interrupts are disabled if the
// function disables the master.  Returns non-zero if the master was already enabled when the
// function was called; zero if the master was disabled at call-time.
//
uint8_t twi_master_enable(const uint8_t enable)
{
    const uint8_t ret = TWI0_MCTRLA & TWI_ENABLE_bm;

    if(enable)
    {
        // Enable the TWI master and force the bus to the "idle" state
        TWI0_MCTRLA |= TWI_ENABLE_bm | TWI_RIEN_bm | TWI_WIEN_bm;
        TWI0_MSTATUS = (TWI0_MSTATUS & ~TWI_BUSSTATE_gm) | TWI_BUSSTATE_IDLE_gc;
    }
    else
        TWI0_MCTRLA &= ~(TWI_ENABLE_bm | TWI_RIEN_bm | TWI_WIEN_bm);

    return ret;
}


// twi_cmd_get_state() - return the current state of the asynchronous command processor.
//
TWICmdState_t twi_cmd_get_state()
{
    return twi_command.state;
}


// twi_cmd_state_busy() - return non-zero if a command is currently in progress, or zero if any of
// the following conditions are true: no command is in progress, a command failed because a NACK
// occurred, a command failed for any other reason.
//
uint8_t twi_cmd_state_busy()
{
    const TWICmdState_t state = twi_cmd_get_state();

    return ((state != TWICmdStateIdle) &&
            (state != TWICmdStateNack) &&
            (state != TWICmdStateError));
}


// twi_cmd_get_data() - get the data value associated with a command.  In the case of write
// commands, the value to be written will be returned.  In the case of read commands, and assuming
// that the command has completed successfully, the value read from the slave device will be
// returned.
//
uint8_t twi_cmd_get_data()
{
    return twi_command.data;
}


// twi_cmd_reset_state() - reset the current command state to "idle".  Should only be called to
// clear a NACK/error condition in the previous command.
//
void twi_cmd_reset_state()
{
    twi_command.state = TWICmdStateIdle;
}


// twi_cmd_start() - helper function for twi_(read|write)_register().  Initiates an asynchronous
// read (if <is_write> equals zero) or write (if <is_write> is non-zero) command specifying the
// device with address <dev_addr> containing the register identified by <reg_addr>.  The <data>
// argument contains data to be written in a write command, and is ignore for read commands.
// Returns a value from the TWICmdStatus_t enumeration to indicate the outcome of command
// initiation.
//
static TWICmdStatus_t twi_cmd_start(const uint8_t dev_addr, const uint8_t reg_addr,
                                    const uint8_t data, const uint8_t is_write)
{
    if(twi_command.state != TWICmdStateIdle)
        return TWICmdBusy;

    if(twi_bus_status() != TWIBusStateIdle)
        return TWICmdBusBusy;

    twi_command.dev_addr = dev_addr;
    twi_command.reg_addr = reg_addr;

    if(is_write)
    {
        twi_command.write = 1;
        twi_command.data = data;
    }
    else
        twi_command.write = 0;

    TWI0_MADDR = (twi_command.dev_addr << 1) | TWI_CMD_WRITE;
    twi_command.state = TWICmdDevAddr;

    return TWICmdSuccess;
}


// twi_sync_cmd() - helper for the twi_(read|write)_register_sync() functions.  Performs a read or
// write command synchronously.  Returns a value from the TWICmdStatus_t enumeration to indicate
// the outcome of the command.
//
static TWICmdStatus_t twi_sync_cmd(const uint8_t dev_addr, const uint8_t reg_addr,
                                   uint8_t * const data, const uint8_t is_write)
{
    TWICmdStatus_t status;

    while(twi_cmd_state_busy())
        ;                           // Wait for any pending commands to complete

    while(twi_bus_status() != TWIBusStateIdle)
        ;                           // Wait for the TWI bus to become idle

    status = is_write ? twi_register_write(dev_addr, reg_addr, *data) :
                        twi_register_read(dev_addr, reg_addr);
    if(status != TWICmdSuccess)
        return status;

    while(twi_cmd_state_busy())
        ;                           // Wait for the command to complete

    status = twi_cmd_get_state();

    switch(status)
    {
        case TWICmdSuccess:
            if(!is_write)
                *data = twi_cmd_get_data();
            break;

        case TWICmdStateNack:
        case TWICmdStateError:
            // If the slave device didn't acknowledge us, or the command failed for any other
            // reason, reset the state of the command object so that another command can be
            // initiated.
            twi_cmd_reset_state();
            break;

        default:
            // Do nothing
            break;
    }

    return status;
}


// twi_register_read() - initiate an asynchronous read of the register with address <reg_addr> in
// the device with address <dev_addr>.  Returns a value from the TWICmdStatus_t enumeration to
// indicate the outcome of command initiation.
//
TWICmdStatus_t twi_register_read(const uint8_t dev_addr, const uint8_t reg_addr)
{
    return twi_cmd_start(dev_addr, reg_addr, 0, 0);
}


// twi_register_write() - initiate an asynchronous write of the register with address <reg_addr> in
// the device with address <dev_addr>.  Returns a value from the TWICmdStatus_t enumeration to
// indicate the outcome of command initiation.
//
TWICmdStatus_t twi_register_write(const uint8_t dev_addr, const uint8_t reg_addr, const uint8_t data)
{
    return twi_cmd_start(dev_addr, reg_addr, data, 1);
}


// twi_sync_register_read() - perform a synchronous read of the register with address <reg_addr> in
// the device with the address <dev_addr>.  If the read is successful, the value read from the
// register will be returned through <data>.  Returns a value from the TWICmdStatus_t enumeration
// to indicate the outcome of the command.
//
TWICmdStatus_t twi_sync_register_read(const uint8_t dev_addr, const uint8_t reg_addr,
                               uint8_t * const data)
{
    return twi_sync_cmd(dev_addr, reg_addr, data, 0);
}


// twi_sync_register_write() - perform a synchronous write of the data in <data> to the register
// with address <reg_addr> in the device with the address <dev_addr>.  Returns a value from the
// TWICmdStatus_t enumeration to indicate the outcome of the command.
//
TWICmdStatus_t twi_sync_register_write(const uint8_t dev_addr, const uint8_t reg_addr,
                                       uint8_t data)
{
    return twi_sync_cmd(dev_addr, reg_addr, &data, 1);
}


// twi_set_clock() - configure the TWI clock to operate at the rate specified by <speed>.  This
// has the effect of setting the TWI clock divisor to suit the current peripheral clock frequency.
// Returns non-zero on success, zero on failure.  As required by the TWI peripheral, this function
// will disable the TWI master while the clock rate is changed.  The master will always be left in
// the same enabled/disabled state as it was before the function was called.  To guarantee this
// behaviour, any interrupts which may change the enabled/disabled state of the TWI peripheral must
// be disabled around calls to this function.
//
uint8_t twi_set_clock(const TWISpeed_t speed)
{
    uint32_t bus_freq, pclk_freq;
    uint16_t baud_val;
    uint8_t was_enabled;

    pclk_freq = pclk_get_freq();
    if(!pclk_freq)
        return 0;

    was_enabled = twi_master_enable(0);

    if(speed == TWISpeed_400kHz)
        bus_freq = 400e3;
    else
        bus_freq = 100e3;

    // Calculate value for the MBAUD register.  Add 1 to the value to account for integer rounding;
    // this is cheesy, but ensures that the specified baud rates are not exceeded.
    baud_val = 1 + (((pclk_freq / bus_freq) - 10) / 2);

    TWI0_MBAUD = baud_val;
    twi_master_enable(was_enabled);

    return 1;
}


// twi_bus_status() - obtain the status of the TWI bus and return it in the form of a TWIBusState_t
// enum.
//
static TWIBusState_t twi_bus_status()
{
    return (TWIBusState_t) (TWI0_MSTATUS & TWI_BUSSTATE_gm);
}
