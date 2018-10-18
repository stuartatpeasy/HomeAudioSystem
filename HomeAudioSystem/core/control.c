/*
    control.c - definitions relating to the RS-485 control interface between the home audio system
    controller and its various connected modules.

    Stuart Wallace <stuartw@atom.net>, October 2018.
*/

#include "control.h"
#include "dev/sc16is752.h"
#include "util/irq.h"
#include <stddef.h>             // size_t


// CtrlCmd_t - enumeration of commands supported by the system
//
typedef enum CtrlCmd
{
    CmdIdentify     = 0,
    CmdPowerState   = 1,
    CmdChannel      = 2,
    CmdGain         = 3,
    CmdPairing      = 4
} CtrlCmd_t;


// CtrlMsgHeader_t - header block, which starts all command and command-response packets.
//
typedef struct CtrlMsgHeader
{
    uint8_t             type_hop;       // [7:4] - message type; [2:0] - hop count
    uint8_t             id;             // Message ID
    CtrlCmd_t           cmd;            // Command identifier
    uint8_t             cksum;          // Checksum
} CtrlMsgHeader_t;


// CtrlCmdArg_t - union of the various one-byte arguments that can be supplied in command packets.
//
typedef union CtrlCmdArg
{
    uint8_t                 block;          // CmdIdentify: block number
    CtrlArgPowerState_t     power_state;    // CmdPowerState: desired power state
    CtrlArgChannel_t        channel;        // CmdChannel: channel ID
    int8_t                  gain;           // CmdGain: desired gain, in units of 0.5dB
    CtrlArgPairingState_t   pairing_state;  // CmdPairing: arguments related to pairing state
} CtrlCmdArg_t;


// CtrlMsgCommand_t - structure defining a control command, sent by the controller module to a
// peripheral module.
//
typedef struct CtrlMsgCommand
{
    CtrlMsgHeader_t     header;
    CtrlCmdArg_t        arg;
} CtrlMsgCommand_t;


// CtrlMsgResponse_t - structure defining a control command response, sent by a peripheral to the
// controller.
//
typedef struct CtrlMsgResponse
{
    CtrlMsgHeader_t     header;
    CtrlResponse_t      resp;
} CtrlMsgResponse_t;


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

static void ctrl_fsm_downstream(CtrlMsgCommand_t * const cmd, const uint8_t rx_char);
static void ctrl_handle_command_packet(const CtrlMsgCommand_t * const cmd);

static const SC16IS752Channel_t ChanDownstream = SC16IS752ChannelA,
                                ChanUpstream = SC16IS752ChannelB;


// ctrl_init() - initialise the control interface
//
void ctrl_init()
{
    g_flags = 0;

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
    static CtrlMsgCommand_t downstream_packet;

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
            ctrl_fsm_downstream(&downstream_packet, sc16is752_rx(ChanDownstream));

        g_flags &= ~CTRL_FLAG_SERIAL_IRQ;
        interrupt_enable_increment();
    }

    if(g_flags & CTRL_FLAG_DOWNSTREAM_PKT_RX)
    {
        if(downstream_packet.header.type_hop & CMD_HOP_MASK)
        {
            // Decrement hop count and forward packet to upstream port
            --downstream_packet.header.type_hop;

            // FIXME - check for failure
            sc16is752_tx_buf(ChanUpstream, &downstream_packet, sizeof(downstream_packet));
        }
        else
            ctrl_handle_command_packet(&downstream_packet);

        g_flags &= ~CTRL_FLAG_DOWNSTREAM_PKT_RX;
    }
}


// ctrl_fsm_downstream() - FSM handling receipt of data from the downstream control port.
//
static void ctrl_fsm_downstream(CtrlMsgCommand_t * const cmd, const uint8_t rx_char)
{
    static CtrlFSMCommand_t state = CtrlFSMWaitStart1;
    static uint8_t cksum;

    switch(state)
    {
        case CtrlFSMWaitStart1:
            if(rx_char == CTRL_PACKET_START1)
                state = CtrlFSMWaitStart2;
            break;

        case CtrlFSMWaitStart2:
            // If <rx_char> equals the second start byte, prepare to receive the packet; otherwise
            // reset the FSM such that it waits for the start of a new packet.
            if(rx_char == CTRL_PACKET_START2)
            {
                state = CtrlFSMWaitCmdByte1;
                cksum = 0;
            }
            else
                state = CtrlFSMWaitStart1;
            break;

        case CtrlFSMWaitCmdByte1:
        case CtrlFSMWaitCmdByte2:
        case CtrlFSMWaitCmdByte3:
        case CtrlFSMWaitCmdByte4:
        case CtrlFSMWaitCmdByte5:
            *(((uint8_t *) cmd) + (state - CtrlFSMWaitCmdByte1))
                = rx_char;
            cksum += rx_char;
            ++state;
            break;

        case CtrlFSMWaitStop:
            // If a stop byte is received, and the packet checksum is valid, set the bit in g_flags
            // indicating that a packet has been successfully received.  In all cases, reset the
            // FSM so that a new packet may be received.
            // NOTE: the received packet must be handled (or copied) before the arrival of the
            // first data byte of any subsequent packet.
            if((rx_char == CTRL_PACKET_STOP) && (cksum == CTRL_CKSUM))
                g_flags |= CTRL_FLAG_DOWNSTREAM_PKT_RX;

            state = CtrlFSMWaitStart1;
            break;
    }
}


// ctrl_handle_command_packet() - handle a packet addressed to this device by the controller.
//
static void ctrl_handle_command_packet(const CtrlMsgCommand_t * const cmd)
{
    CtrlMsgResponse_t response;
    size_t len = sizeof(response);
    const uint8_t start[2] = {CTRL_PACKET_START1, CTRL_PACKET_START2};

    response.resp = CtrlRespUnsupportedOperation;

    switch(cmd->header.cmd)
    {
        case CmdIdentify:
            break;

        case CmdPowerState:
            break;

        case CmdChannel:
            response.resp = ctrl_set_channel(cmd->arg.channel);
            break;

        case CmdGain:
            response.resp = ctrl_set_gain(cmd->arg.gain);
            break;

        case CmdPairing:
            response.resp = ctrl_set_pairing(cmd->arg.pairing_state);
            break;
    }

    response.header.cmd = cmd->header.cmd;
    response.header.id = cmd->header.id;
    response.header.type_hop = 0;                           // FIXME check this

    // Transmit response
    // FIXME - check for errors
    sc16is752_tx_buf(ChanDownstream, start, sizeof(start));
    sc16is752_tx_buf(ChanDownstream, &response, len);
    sc16is752_tx(ChanDownstream, CTRL_PACKET_STOP);
}
