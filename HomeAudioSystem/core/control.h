#ifndef MODULES_CONTROL_H_INC
#define MODULES_CONTROL_H_INC
/*
    control.h - declarations relating to the RS-485 control interface between the home audio system
    controller and its various connected modules.

    Stuart Wallace <stuartw@atom.net>, October 2018.
*/


typedef enum CtrlArgChannel
{
    CtrlAmpChannelLeft      = 0,    // For mono speakers: use the left channel
    CtrlAmpChannelRight     = 1,    // For mono speakers: use the right channel
    CtrlAmpChannelLeftRight = 2,    // For stereo speakers: use the left and right channels
    CtrlSrcChannel1         = 3,    // For sources: use S/PDIF channel 1
    CtrlSrcChannel2         = 4     // For sources: use S/PDIF channel 2
} CtrlArgChannel_t;


typedef enum CtrlResponse
{
    CtrlRespOK                      = 0,
    CtrlRespOperationFailed         = 1,
    CtrlRespUnsupportedOperation    = 2,
    CtrlRespBadArg                  = 3
} CtrlResponse_t;


void ctrl_init();
void ctrl_serial_isr();
void ctrl_worker();

// Command-handler function declarations.  These functions should be implemented in the appropriate
// hardware-driver module.
//
CtrlResponse_t ctrl_amp_set_channel(const CtrlArgChannel_t channel);

#endif
