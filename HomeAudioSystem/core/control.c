/*
    control.c - definitions relating to the RS-485 control interface between the home audio system
    controller and its various connected modules.

    Stuart Wallace <stuartw@atom.net>, October 2018.
*/

#include "control.h"
#include "dev/sc16is752.h"
#include "util/irq.h"


#define CTRL_RX_PAYLOAD_BUF_LEN     (31)        // Length of packet RX buffer in bytes

// CtrlCmd_t - enumeration of commands supported by the system
//
typedef enum CtrlCmd
{
    CmdIdentify     = 0,        // Obtain device identity, block by block
    CmdPowerState   = 1,        // Set device power state
    CmdChannel      = 2,        // Set channel amplified by device, or used by source device
    CmdGain         = 3,        // Set device gain
    CmdPairing      = 4,        // Set or retrieve pairing status (e.g. for Bluetooth devices)
    CmdSetName      = 5         // Set device name, to be displayed to users
} CtrlCmd_t;


// CtrlMsgHeader_t - header block, which starts all command and command-response packets.
//
typedef struct CtrlMsgHeader
{
    uint8_t             len_hop;        // [7:3] - payload length; [2:0] - hop count
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


// CtrlFSMCommand_t - FSM states for command receipt.  This FSM is used when receiving data from
// the downstream control port.
//
typedef enum CtrlFSMCommand
{
    CtrlFSMWaitStart1,              // Waiting for first start byte
    CtrlFSMWaitStart2,              // Waiting for second start byte
    CtrlFSMHeader,                  // Receiving the command packet header
    CtrlFSMPayload,                 // Receiving the message payload
    CtrlFSMWaitStop                 // Waiting for stop byte
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

// CtrlMsgHeader_t bit masks
#define CMD_HOP_MASK                (0x07)      // Mask for "hop" part of len_hop field
#define CMD_LEN_MASK                (0xf8)      // Mask for "len" part of len_hop field
#define CMD_LEN_SHIFT               (3)         // Offset of bit 0 of "len" part of len_hop field

// Obtain payload length from a message header
#define CMD_HDR_GET_PAYLOAD_LEN(hdr)    (((hdr).len_hop & CMD_LEN_MASK) >> CMD_LEN_SHIFT)

static volatile uint8_t g_flags;

static void ctrl_fsm_receive_byte(const uint8_t rx_char);
static void ctrl_handle_command_packet();

static const SC16IS752Channel_t ChanDownstream = SC16IS752ChannelA,
                                ChanUpstream = SC16IS752ChannelB;

// Buffer for packets received from downstream controller
static struct
{
    CtrlMsgHeader_t hdr;
    uint8_t payload[CTRL_RX_PAYLOAD_BUF_LEN];
} rx_buffer;


// ctrl_init() - initialise the control interface.
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

    // FIXME - enable RHR interrupts; disable all other types of interrrupt.
}


// ctrl_serial_isr() - ISR called when the SC16IS752 dual UART raises an interrupt.  This function
// does no work; it uses a bit in the <g_flags> global to signal to ctrl_worker() that an interrupt
// needs to be serviced.
//
void ctrl_serial_isr()
{
    g_flags |= CTRL_FLAG_SERIAL_IRQ;
}


// ctrl_worker() - worker "process" for the control interface.  Needs to be called in a fast loop.
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
            ctrl_fsm_receive_byte(sc16is752_rx(ChanDownstream));

        g_flags &= ~CTRL_FLAG_SERIAL_IRQ;
        interrupt_enable_increment();
    }

    if(g_flags & CTRL_FLAG_DOWNSTREAM_PKT_RX)
    {
        if(rx_buffer.hdr.len_hop & CMD_HOP_MASK)
        {
            const uint8_t len = sizeof(CtrlMsgHeader_t) +
                                ((rx_buffer.hdr.len_hop & CMD_LEN_MASK) >> CMD_LEN_SHIFT);

            // Decrement hop count and forward packet to upstream port
            --rx_buffer.hdr.len_hop;

            // FIXME - check for failure
            sc16is752_tx_buf(ChanUpstream, &rx_buffer, len);
        }
        else
            ctrl_handle_command_packet();

        g_flags &= ~CTRL_FLAG_DOWNSTREAM_PKT_RX;
    }
}


// ctrl_fsm_receive_byte() - FSM handling receipt of data from the downstream control port.
//
static void ctrl_fsm_receive_byte(const uint8_t rx_char)
{
    static CtrlFSMCommand_t state = CtrlFSMWaitStart1;
    static uint8_t cksum, bytes_remaining, write_ptr;

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
                state = CtrlFSMHeader;
                bytes_remaining = sizeof(CtrlMsgHeader_t);
                write_ptr = 0;
                cksum = 0;
            }
            else
                state = CtrlFSMWaitStart1;
            break;

        case CtrlFSMHeader:
        case CtrlFSMPayload:
            *(((uint8_t *) &rx_buffer) + write_ptr++) = rx_char;
            cksum += rx_char;
            if(!--bytes_remaining)
            {
                if(state == CtrlFSMHeader)
                {
                    // Header received; now obtain payload length and prepare to receive payload
                    bytes_remaining = CMD_HDR_GET_PAYLOAD_LEN(rx_buffer.hdr);
                    state = CtrlFSMPayload;
                }
                else
                {
                    // Payload received; now wait for stop byte
                    state = CtrlFSMWaitStop;
                }
            }
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
static void ctrl_handle_command_packet()
{
    CtrlMsgResponse_t response;
    size_t len = sizeof(response);
    const size_t payload_len = (rx_buffer.hdr.len_hop & CMD_LEN_MASK) >> CMD_LEN_SHIFT;
    const uint8_t start[2] = {CTRL_PACKET_START1, CTRL_PACKET_START2};
    uint8_t cksum = 0;

    // Each branch of the following switch statement validates the length of the received payload
    // before executing the corresponding command handler.  Default the response code to
    // CtrlRespBadPacketLen here, in order to avoid identical "else" clauses in each branch of the
    // switch.
    response.resp = CtrlRespBadPacketLen;

    switch(rx_buffer.hdr.cmd)
    {
        case CmdIdentify:
            if(payload_len == sizeof(uint8_t))
                response.resp = ctrl_identify(rx_buffer.payload[0]);
            break;

        case CmdPowerState:
            if(payload_len == sizeof(CtrlArgPowerState_t))
                response.resp = ctrl_set_power_state(*((CtrlArgPowerState_t *) rx_buffer.payload));
            break;

        case CmdChannel:
            if(payload_len == sizeof(CtrlArgChannel_t))
                response.resp = ctrl_set_channel(*((CtrlArgChannel_t *) rx_buffer.payload));
            break;

        case CmdGain:
            if(payload_len == sizeof(int8_t))
                response.resp = ctrl_set_gain((int8_t) rx_buffer.payload[0]);
            break;

        case CmdPairing:
            if(payload_len == sizeof(CtrlArgPairingState_t))
                response.resp = ctrl_set_pairing(*((CtrlArgPairingState_t *) rx_buffer.payload));
            break;

        case CmdSetName:
            response.resp = ctrl_set_name((const char *) &rx_buffer.payload, payload_len);
            break;

        default:
            response.resp = CtrlRespUnsupportedOperation;
            break;
    }

    response.header.cmd = rx_buffer.hdr.cmd;
    response.header.id = rx_buffer.hdr.id;
    response.header.len_hop = 0;                           // FIXME check this
    response.header.cksum = 0;

    // Compute checksum
    for(uint8_t i = 0, cksum = 0; i < len; ++i)
        cksum += *((uint8_t *) &response);

    response.header.cksum = CTRL_CKSUM - cksum;

    // Transmit response
    // FIXME - check for errors
    sc16is752_tx_buf(ChanDownstream, start, sizeof(start));
    sc16is752_tx_buf(ChanDownstream, &response, len);
    sc16is752_tx(ChanDownstream, CTRL_PACKET_STOP);
}
