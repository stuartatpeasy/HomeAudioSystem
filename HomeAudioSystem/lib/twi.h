#ifndef LIB_TWI_H_INC
#define LIB_TWI_H_INC
/*
    twi.h - declarations relating to the two-wire interface (TWI) peripheral

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "types.h"
#include <stdint.h>


// TWISpeed_t - enumeration of supported TWI bus speeds.
//
typedef enum TWISpeed
{
    TWISpeed_100kHz = 1,            // "Normal" speed, 100kHz
    TWISpeed_400kHz = 2             // "Fast" speed, 400kHz
} TWISpeed_t;


// TWICmdState_t - state machine for TWI read/write register commands.
//
typedef enum TWICmdState
{
    TWICmdStateIdle = 0,    // Command state is idle
    TWICmdStateNack,        // NACK from slave
    TWICmdStateError,       // Unexpected interrupt or other error condition
    TWICmdDevAddr,          // Waiting for device address to be sent
    TWICmdRegAddr,          // Waiting for register address to be sent
    TWICmdDevAddr2,         // Waiting for device address to be sent (second half of read command)
    TWICmdDataWrite,        // Waiting for data to be sent
    TWICmdDataRead          // Waiting for data to be received
} TWICmdState_t;


// TWICmdStatus_t - enumeration of return values for the publicly-callable functions which initiate
// TWI read/write commands.
//
typedef enum TWICmdStatus
{
    TWICmdSuccess = 0,      // Command successfully initiated
    TWICmdBusy,             // Command could not be initiated because another command is running
    TWICmdBusBusy,          // Command could not be initiated because the TWI bus is busy
    TWICmdNack,             // No acknowledgment from slave device
    TWICmdTimeout,          // Command timed out
    TWICmdError             // An error occurred during command processing
} TWICmdStatus_t;


void twi_configure_master(const Pinset_t pinset);
uint8_t twi_master_enable(const uint8_t enable);
uint8_t twi_set_clock(const TWISpeed_t speed);
TWICmdState_t twi_cmd_get_state();
uint8_t twi_cmd_state_busy();
uint8_t twi_cmd_get_data();
void twi_cmd_reset_state();
TWICmdStatus_t twi_register_read(const uint8_t dev_addr, const uint8_t reg_addr);
TWICmdStatus_t twi_register_write(const uint8_t dev_addr, const uint8_t reg_addr,
                                  const uint8_t data);
TWICmdStatus_t twi_sync_register_read(const uint8_t dev_addr, const uint8_t reg_addr,
                                      uint8_t * const data);
TWICmdStatus_t twi_sync_register_write(const uint8_t dev_addr, const uint8_t reg_addr,
                                       uint8_t data);
// TODO: reset_nack
// TODO: reset_err

#endif
