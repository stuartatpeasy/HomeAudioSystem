#ifndef MODULES_CONTROL_H_INC
#define MODULES_CONTROL_H_INC
/*
    control.h - declarations relating to the RS-485 control interface between the home audio system
    controller and its various connected modules.

    Stuart Wallace <stuartw@atom.net>, October 2018.
*/

#include <stddef.h>             // size_t
#include <stdint.h>


// CtrlArgChannel_t - enumeration of arguments to the "channel" command, which (for amplifier
// modules) sets the audio channel to amplify, and (for source modules) specifies which S/PDIF
// channel to use when sending data.
//
typedef enum CtrlArgChannel
{
    CtrlAmpChannelLeft      = 0,    // For mono speakers: use the left channel
    CtrlAmpChannelRight     = 1,    // For mono speakers: use the right channel
    CtrlAmpChannelLeftRight = 2,    // For stereo speakers: use the left and right channels
    CtrlSrcChannel1         = 3,    // For sources: use S/PDIF channel 1
    CtrlSrcChannel2         = 4     // For sources: use S/PDIF channel 2
} CtrlArgChannel_t;


// CtrlArgPairingState_t - enumeration of arguments to the "pairing" command, which sets or
// retrieves the pairing state for audio source devices which support pairing (e.g. Bluetooth
// modules).
//
typedef enum CtrlArgPairingState
{
    ArgPairingGetState      = 0,    // Respond with the current pairing state
    ArgPairingForceUnpair   = 1     // Unpair from any paired audio source
} CtrlArgPairingState_t;


// CtrlArgPowerState_t - enumeration of arguments to the "power state" command, which sets the
// power state of a module.
//
typedef enum CtrlArgPowerState
{
    ArgPowerStateMinimum    = 0,    // Enter the lowest-possible power state
    ArgPowerStateSleep      = 1,    // Enter a sleep state
    ArgPowerStateFull       = 2     // Enter full-power, normal-operations state
} CtrlArgPowerState_t;


// CtrlResponse_t - enumeration of control command response codes
//
typedef enum CtrlResponse
{
    CtrlRespOK                      = 0,    // The operation completed successfully
    CtrlRespOperationFailed         = 1,    // The operation failed
    CtrlRespUnsupportedOperation    = 2,    // The operation is not supported by this device
    CtrlRespBadArg                  = 3,    // The command supplied an invalid argument
    CtrlRespBadPacketLen            = 4     // The command has an invalid length
} CtrlResponse_t;


void ctrl_init();
void ctrl_serial_isr();
void ctrl_worker();


// Command-handler function declarations.  These functions should be implemented in the appropriate
// hardware-driver module.
//
CtrlResponse_t ctrl_identify(const uint8_t block);
CtrlResponse_t ctrl_set_power_state(const CtrlArgPowerState_t state);
CtrlResponse_t ctrl_set_channel(const CtrlArgChannel_t channel);
CtrlResponse_t ctrl_set_gain(const int8_t gain);
CtrlResponse_t ctrl_set_pairing(const CtrlArgPairingState_t state);
CtrlResponse_t ctrl_set_name(const char * const name, const size_t len);

#endif
