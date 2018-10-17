/*
    control.c - definitions relating to the RS-485 control interface between the home audio system
    controller and its various connected modules.

    Stuart Wallace <stuartw@atom.net>, October 2018.
*/

#include "control.h"
#include "../dev/sc16is752.h"
#include "../util/irq.h"
#include <stdint.h>


typedef enum CtrlCmd
{
    CmdIdentify     = 0,
    CmdPowerState   = 1,
    CmdChannel      = 2,
    CmdVolume       = 3,
    CmdPairingState = 4
} CtrlCmd_t;


typedef enum CtrlArgPowerState
{
    ArgPowerStateMinimum    = 0,
    ArgPowerStateFull       = 1
} CtrlArgPowerState_t;


typedef enum CtrlArgChannel
{
    ArgChannelOutLeft       = 0,        // For mono speakers: use the left channel
    ArgChannelOutRight      = 1,        // For mono speakers: use the right channel
    ArgChannelIn1           = 2,        // For sources: use S/PDIF channel 1
    ArgChannelIn2           = 3         // For sources: use S/PDIF channel 2
} CtrlArgChannel_t;


typedef struct CtrlMessage
{
    uint8_t     type_hop;       // [7:4] - message type; [2:0] - hop count
    uint8_t     id;             // Message ID
    CtrlCmd_t   cmd;            // Command identifier

    union
    {
        uint8_t             block;
        CtrlArgPowerState_t state;
        CtrlArgChannel_t    channel;
        uint8_t             volume;
    } arg;

    uint8_t     cksum;          // Checksum
} CtrlMessage_t;

#define CMD_HOP_MASK    (0x07)
#define CMD_TYPE_MASK   (0xf0)

#define CTRL_FLAG_SERIAL_IRQ        (0x01)

static volatile uint8_t g_flags;


void ctrl_init()
{
    g_flags = 0;

    sc16is752_init();                       // Initialise the SC16IS752 dual UART

    // Configure control channel baud rate, and enable automatic transceiver direction-switching
    // using the SC16IS752's RS-485 auto-nRTS feature.
    for(SC16IS752Channel_t c = SC16IS752ChannelA; c <= SC16IS752ChannelB;
        c += SC16IS752ChannelOffset)
    {
        sc16is752_set_baud_rate(c, 460800);
        sc16is752_enable_auto_rts(c, 1, 1);
    }
}


// ctrl_serial_isr() - ISR called when the SC16IS752 dual UART raises an interrupt.  This function
// does no work; it uses a bit in the <g_flags>
void ctrl_serial_isr()
{
    g_flags |= CTRL_FLAG_SERIAL_IRQ;
}


void ctrl_worker()
{
    if(g_flags & CTRL_FLAG_SERIAL_IRQ)
    {
        SC16IS752IntFlags_t ser_irq_flags;

        interrupt_enable_decrement();

        ser_irq_flags = sc16is752_get_interrupt_source(SC16IS752ChannelA);
        if(SC16IS752IntRXDataReady(ser_irq_flags))
        {
            // Handle data received from downstream (?) port
        }

        ser_irq_flags = sc16is752_get_interrupt_source(SC16IS752ChannelB);

        g_flags &= ~CTRL_FLAG_SERIAL_IRQ;
        interrupt_enable_increment();
    }
}
