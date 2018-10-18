/*
    control.c - definitions relating to the RS-485 control interface between the home audio system
    controller and its various connected modules.

    Stuart Wallace <stuartw@atom.net>, October 2018.
*/

#include "control.h"
#include "dev/sc16is752.h"
#include "util/irq.h"
#include <stddef.h>             // size_t
#include <stdint.h>


// CtrlCmd_t - enumeration of commands supported by the system
//
typedef enum CtrlCmd
{
    CmdIdentify     = 0,
    CmdPowerState   = 1,
    CmdChannel      = 2,
    CmdVolume       = 3,
    CmdPairing      = 4
} CtrlCmd_t;


typedef enum CtrlArgPowerState
{
    ArgPowerStateMinimum    = 0,
    ArgPowerStateFull       = 1
} CtrlArgPowerState_t;


typedef enum CtrlArgPairingState
{
    ArgPairingGetState      = 0,
    ArgPairingForceUnpair   = 1
} CtrlArgPairingState_t;


typedef struct CtrlMsgHeader
{
    uint8_t             type_hop;       // [7:4] - message type; [2:0] - hop count
    uint8_t             id;             // Message ID
    CtrlCmd_t           cmd;            // Command identifier
    uint8_t             cksum;          // Checksum
} CtrlMsgHeader_t;


typedef union CtrlCmdArg
{
    uint8_t             block;          // CmdIdentify: block number
    CtrlArgPowerState_t state;          // CmdPowerState: desired power state
    CtrlArgChannel_t    channel;        // CmdChannel: channel ID
    uint8_t             volume;         // CmdVolume: desired volume level
} CtrlCmdArg_t;


typedef struct CtrlMsgCommand
{
    CtrlMsgHeader_t     header;
    CtrlCmdArg_t        arg;
} CtrlMsgCommand_t;



#define CMD_HOP_MASK    (0x0f)
#define CMD_TYPE_MASK   (0xf0)

// CtrlFSMCommand_t - FSM states for command receipt.  This FSM is used when receiving data from
// the downstream control port.
//
typedef enum CtrlFSMCommand
{
    CtrlFSMWaitStart1,
    CtrlFSMWaitStart2,
    CtrlFSMWaitCmdByte1,
    CtrlFSMWaitCmdByte2,
    CtrlFSMWaitCmdByte3,
    CtrlFSMWaitCmdByte4,
    CtrlFSMWaitCmdByte5,
    CtrlFSMWaitStop
} CtrlFSMCommand_t;

// Start and stop bytes for control packets
#define CTRL_PACKET_START1          (0xAA)      // First start byte of a command packet
#define CTRL_PACKET_START2          (0xCC)      // Second start byte of a command packet
#define CTRL_PACKET_STOP            (0xBB)      // Command packet stop byte

// Flag bits used in g_flags
#define CTRL_FLAG_SERIAL_IRQ        (0x01)      // Serial port controller has raised an interrupt
#define CTRL_FLAG_DOWNSTREAM_PKT_RX (0x02)      // Packet received from downstream

#define CTRL_BAUD_RATE              (460800)    // Baud rate of control interface
#define CTRL_CKSUM                  (0xff)      // Checksum of a valid packet

static volatile uint8_t g_flags;
static uint8_t g_downstream_cksum;
static CtrlFSMCommand_t g_downstream_state;
static CtrlMsgCommand_t g_downstream_packet;

static void ctrl_fsm_downstream(const uint8_t rx_char);
static void ctrl_handle_command_packet();

static const SC16IS752Channel_t ChanDownstream = SC16IS752ChannelA,
                                ChanUpstream = SC16IS752ChannelB;


// ctrl_init() - initialise the control interface
//
void ctrl_init()
{
    g_flags = 0;
    g_downstream_state = CtrlFSMWaitStart1;

    sc16is752_init();           // Initialise the SC16IS752 dual UART

    // Configure control channel baud rate, and enable automatic transceiver direction-switching
    // using the SC16IS752's RS-485 auto-nRTS feature.
    for(SC16IS752Channel_t c = SC16IS752ChannelA; c <= SC16IS752ChannelB;
        c += SC16IS752ChannelOffset)
    {
        sc16is752_set_baud_rate(c, CTRL_BAUD_RATE);     // FIXME - check for errors
        sc16is752_enable_auto_rts(c, 1, 1);
    }
}


// ctrl_serial_isr() - ISR called when the SC16IS752 dual UART raises an interrupt.  This function
// does no work; it uses a bit in the <g_flags>
void ctrl_serial_isr()
{
    g_flags |= CTRL_FLAG_SERIAL_IRQ;
}


// ctrl_worker() -
//
void ctrl_worker()
{
    if(g_flags & CTRL_FLAG_SERIAL_IRQ)
    {
        SC16IS752IntFlags_t ser_irq_flags;

        interrupt_enable_decrement();

        // Handle interrupts from the upstream port (channel B)
        ser_irq_flags = sc16is752_get_interrupt_source(ChanUpstream);

        // If a byte has been received from the upstream port, forward it to the downstream port.
        if(SC16IS752IntRXDataReady(ser_irq_flags))
            sc16is752_tx(ChanDownstream, sc16is752_rx(ChanUpstream));

        // Handle interrupts from the downstream port (channel A)
        ser_irq_flags = sc16is752_get_interrupt_source(ChanDownstream);

        // If a byte has been received from the downstream port, pass it to the downstream FSM for
        // handling.
        if(SC16IS752IntRXDataReady(ser_irq_flags))
            ctrl_fsm_downstream(sc16is752_rx(ChanDownstream));

        g_flags &= ~CTRL_FLAG_SERIAL_IRQ;
        interrupt_enable_increment();
    }

    if(g_flags & CTRL_FLAG_DOWNSTREAM_PKT_RX)
    {
        if(g_downstream_packet.header.type_hop & CMD_HOP_MASK)
        {
            // Decrement hop count and forward packet to upstream port
            --g_downstream_packet.header.type_hop;

            // FIXME - check for failure
            sc16is752_tx_buf(ChanUpstream, &g_downstream_packet, sizeof(g_downstream_packet));
        }
        else
            ctrl_handle_command_packet();

        g_flags &= ~CTRL_FLAG_DOWNSTREAM_PKT_RX;
    }
}


// ctrl_fsm_downstream() - FSM handling receipt of data from the downstream control port.
//
static void ctrl_fsm_downstream(const uint8_t rx_char)
{
    switch(g_downstream_state)
    {
        case CtrlFSMWaitStart1:
            if(rx_char == CTRL_PACKET_START1)
                g_downstream_state = CtrlFSMWaitStart2;
            break;

        case CtrlFSMWaitStart2:
            // If <rx_char> equals the second start byte, prepare to receive the packet; otherwise
            // reset the FSM such that it waits for the start of a new packet.
            if(rx_char == CTRL_PACKET_START2)
            {
                g_downstream_state = CtrlFSMWaitCmdByte1;
                g_downstream_cksum = 0;
            }
            else
                g_downstream_state = CtrlFSMWaitStart1;
            break;

        case CtrlFSMWaitCmdByte1:
        case CtrlFSMWaitCmdByte2:
        case CtrlFSMWaitCmdByte3:
        case CtrlFSMWaitCmdByte4:
        case CtrlFSMWaitCmdByte5:
            *(((uint8_t *) &g_downstream_packet) + (g_downstream_state - CtrlFSMWaitCmdByte1))
                = rx_char;
            g_downstream_cksum += rx_char;
            ++g_downstream_state;
            break;

        case CtrlFSMWaitStop:
            // If a stop byte is received, and the packet checksum is valid, set the bit in g_flags
            // indicating that a packet has been successfully received.  In all cases, reset the
            // FSM so that a new packet may be received.
            // NOTE: the received packet must be handled (or copied) before the arrival of the
            // first data byte of any subsequent packet.
            if((rx_char == CTRL_PACKET_STOP) && (g_downstream_cksum == CTRL_CKSUM))
                g_flags |= CTRL_FLAG_DOWNSTREAM_PKT_RX;

            g_downstream_state = CtrlFSMWaitStart1;
            break;
    }
}


// ctrl_handle_command_packet() - handle a packet addressed to this device by the controller.
//
static void ctrl_handle_command_packet()
{
    CtrlResponse_t resp;
    size_t resp_len = sizeof(g_downstream_packet.header);

    switch(g_downstream_packet.header.cmd)
    {
        case CmdIdentify:
            break;

        case CmdPowerState:
            break;

        case CmdChannel:
            resp = ctrl_amp_set_channel(g_downstream_packet.arg.channel);
            break;

        case CmdVolume:
            break;

        case CmdPairing:
            break;
    }

//    sc16is752_tx_buf(ChanDownstream, )
}
