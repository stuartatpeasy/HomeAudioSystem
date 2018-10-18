/*
    sc16is752.c - definitions relating to the driver for the SC16IS752 dual SPI/I2C UART IC.

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "sc16is752.h"
#include "lib/debug.h"
#include "lib/spi.h"
#include "platform.h"           // for F_UART
#include "util/irq.h"


// SC16IS752Register_t - enumeration of the registers in the SC16IS752.  Many of these registers
// share an address.  In such cases, bits in other registers determine which register is actually
// accessed.
//
typedef enum SC16IS752Register
{
    SC16IS752RegRHR         = (0x00 << 3),  // [R ] Receive holding register
    SC16IS752RegTHR         = (0x00 << 3),  // [ W] Transmit holding register
    SC16IS752RegDLL         = (0x00 << 3),  // [RW] Baud rate divider - least-significant byte
    SC16IS752RegIER         = (0x01 << 3),  // [RW] Interrupt enable register
    SC16IS752RegDLH         = (0x01 << 3),  // [RW] Baud rate divider - most-significant byte
    SC16IS752RegIIR         = (0x02 << 3),  // [R ] Interrupt identification register
    SC16IS752RegFCR         = (0x02 << 3),  // [ W] FIFO control register
    SC16IS752RegEFR         = (0x02 << 3),  // [RW] Enhanced features register
    SC16IS752RegLCR         = (0x03 << 3),  // [RW] Line control register
    SC16IS752RegMCR         = (0x04 << 3),  // [RW] Modem control register
    SC16IS752RegXON1        = (0x04 << 3),  // [RW] XON1 word
    SC16IS752RegLSR         = (0x05 << 3),  // [R ] Line status register
    SC16IS752RegXON2        = (0x05 << 3),  // [RW] XON2 word
    SC16IS752RegMSR         = (0x06 << 3),  // [R ] Modem status register
    SC16IS752RegTCR         = (0x06 << 3),  // [RW] Transmission control register
    SC16IS752RegXOFF1       = (0x06 << 3),  // [RW] XOFF1 word
    SC16IS752RegSPR         = (0x07 << 3),  // [RW] Scratch pad register
    SC16IS752RegTLR         = (0x07 << 3),  // [RW] Trigger level register
    SC16IS752RegXOFF2       = (0x07 << 3),  // [RW] XOFF2 word
    SC16IS752RegTXLVL       = (0x08 << 3),  // [R ] Transmitter FIFO level register
    SC16IS752RegRXLVL       = (0x09 << 3),  // [R ] Receiver FIFO level register
    SC16IS752RegIODir       = (0x0a << 3),  // [RW] Programmable IO pins direction register
    SC16IS752RegIOState     = (0x0b << 3),  // [RW] Programmable IO pins state register
    SC16IS752RegIOIntEna    = (0x0c << 3),  // [RW] Programmable IO pins interrupt enable register
    SC16IS752RegIOControl   = (0x0e << 3),  // [RW] Programmable IO pins control register
    SC16IS752RegEFCR        = (0x0f << 3),  // [RW] Extra features control register
    SC16IS752Reg_End                        // [--] End-of-list sentinel
} SC16IS752Register_t;

// Macro defining the size of the "stride" between registers, to simplify iterating.
#define SC16IS752RegOffset      (1 << 3)

// Magic number which, when written to LCR, enables the enhanced registers.
#define SC16IS752_ENABLE_ENHANCED_REG   (0xbf)

// SC16IS752FCR (0x02) - FIFO control register - bit definitions
#define SC16IS752_FCR_RX_TRIG_MASK  (3 << 6)    // Mask for the bits defining RX FIFO trigger level
#define SC16IS752_FCR_TX_TRIG_MASK  (3 << 4)    // Mask for the bits defining TX FIFO trigger level
#define SC16IS752_FCR_RESET_TX_FIFO (1 << 2)    // Initiate reset of TX FIFO
#define SC16IS752_FCR_RESET_RX_FIFO (1 << 1)    // Initiate reset of RX FIFO
#define SC16IS752_FCR_FIFO_ENABLE   (1 << 0)    // Enable TX and RX FIFOs

// SC16IS752RXTrigger_t - enumeration of possible values of the "RX trigger level" bits in FCR
typedef enum SC16IS752RXTrigger
{
    SC16IS752RXTrigger8     = (0 << 6),         // RX FIFO trigger level = 8 characters
    SC16IS752RXTrigger16    = (1 << 6),         // RX FIFO trigger level = 16 characters
    SC16IS752RXTrigger56    = (2 << 6),         // RX FIFO trigger level = 56 characters
    SC16IS752RXTrigger60    = (3 << 6)          // RX FIFO trigger level = 60 characters
} SC16IS752RXTrigger_t;

// SC16IS752TXTrigger_t - enumeration of possible values of the "TX trigger level" bits in FCR
typedef enum SC16IS752TXTrigger
{
    SC16IS752TXTrigger8     = (0 << 4),         // TX FIFO trigger level = 8 spaces
    SC16IS752TXTrigger16    = (1 << 4),         // TX FIFO trigger level = 16 spaces
    SC16IS752TXTrigger32    = (2 << 4),         // TX FIFO trigger level = 32 spaces
    SC16IS752TXTrigger56    = (3 << 4)          // TX FIFO trigger level = 56 spaces
} SC16IS752TXTrigger_t;


// SC16IS752RegEFR (0x02) - enhanced features register - bit definitions
#define SC16IS752_EFR_CTS           (1 << 7)    // Enable CTS flow control
#define SC16IS752_EFR_RTS           (1 << 6)    // Enable RTS flow control
#define SC16IS752_EFR_SPECIAL_CHAR  (1 << 5)    // Enable special-character detection
#define SC16IS752_EFR_ENABLE        (1 << 4)    // Enable enhanced functions

// SC16IS752RegLCR (0x03) - line control register - bit definitions
#define SC16IS752_LCR_DIV_LATCH_EN  (1 << 7)    // Divisor latch enable
#define SC16IS752_LCR_BREAK_CTRL    (1 << 6)    // Break control: force a break to be sent
#define SC16IS752_LCR_PARITY_FORCE  (1 << 5)    // If enabled, force parity to inverse of this bit
#define SC16IS752_LCR_PARITY_TYPE   (1 << 4)    // 0 = odd, 1 = even parity (if parity enabled)
#define SC16IS752_LCR_PARITY_ENABLE (1 << 3)    // Enable parity generation and checking
#define SC16IS752_LCR_NSTOP         (1 << 2)    // Number of stop bits: 0 = 1/1.5 bits, 1 = 2 bits
#define SC16IS752_LCR_WORD_LEN_MASK (3 << 0)    // Word length bit-mask

// SC16IS752LCRWordLen_t - enumeration of allowable word lengths in the LCR
typedef enum SC16IS752LCRWordLen
{
    SC16IS752WordLen5 = 0,      // Word length = 5 bits
    SC16IS752WordLen6 = 1,      // Word length = 6 bits
    SC16IS752WordLen7 = 2,      // Word length = 7 bits
    SC16IS752WordLen8 = 3       // Word length = 8 bits
} SC16IS752LCRWordLen_t;

// SC16IS752RegIIR - interrupt identification register - bit definitions
#define SC16IS752_IIR_PRI_MASK      (0x3e)      // Mask for interrupt priority bits
#define SC16IS752_IIR_PRI_SHIFT     (1)         // Offset of interrupt priority bits
#define SC16IS752_IIR_INT_STATUS    (1 << 0)    // Interrupt status: 1 = int pending; 0 = no int

// SC16IS752RegIOControl (0xe) - bit definitions
#define SC16IS752_IOCTRL_SRESET     (1 << 3)    // Software reset
#define SC16IS752_IOCTRL_GPIO30     (1 << 2)    // 0 = GPIO[3..0] are IO; 1 = modem control pins
#define SC16IS752_IOCTRL_GPIO74     (1 << 1)    // 0 = GPIO[7..4] are IO; 1 = modem control pins
#define SC16IS752_IOCTRL_IOLATCH    (1 << 0)    // Enable/disable input interrupt latching

// SC16IS752RegEFCR (0x0f) - extra features control register - bit definitions
#define SC16IS752_EFCR_IRDA_MODE    (1 << 7)    //
#define SC16IS752_EFCR_RTSINVER     (1 << 5)    // Invert nRTS signal in RS-485 auto-RTS mode
#define SC16IS752_EFCR_RTSCON       (1 << 4)    // Enable transmitter to control nRTS pin
#define SC16IS752_EFCR_TXDISABLE    (1 << 2)    // Disable transmitter; FIFO will continue to fill
#define SC16IS752_EFCR_RXDISABLE    (1 << 1)    // Disable receiver
#define SC16IS752_EFCR_9BIT         (1 << 0)    // Enable 9-bit or multi-drop mode (RS-485)


// Declarations relating to the composition of the first byte sent during a SPI transaction.  This
// byte contains the read/write flag, the register address, and a channel specifier.
typedef enum SC16IS752SPIDir
{
    SC16IS752SPIDirWrite    = (0 << 7),             // SPI transaction is a write
    SC16IS752SPIDirRead     = (1 << 7)              // SPI transaction is a read
} SC16IS752SPIDir_t;


static uint8_t sc16is752_read(const SC16IS752Channel_t channel, const SC16IS752Register_t reg);
static void sc16is752_write(const SC16IS752Channel_t channel, const SC16IS752Register_t reg,
                            const uint8_t data);
static uint8_t sc16is752_spi_op(const SC16IS752SPIDir_t dir, const SC16IS752Channel_t channel,
                                const SC16IS752Register_t reg, const uint8_t data);


// sc16is752_init() - initialise the SPI interface and attempt to reset the SC16IS752 device.
// Returns non-zero on success; zero otherwise.
//
uint8_t sc16is752_init()
{
    debug_putstr_p("SC16IS752: init start\n");

    spi0_configure_master(PinsetDefault, SPIClkDiv4);
    spi0_port_activate(1);
    spi0_enable(1);

    sc16is752_reset();

    // Disable GPIO interrupts, set all GPIO pins to logic 0 and configure the pins as outputs.  A
    // side-effect of sc16is752_reset() is to configure all of the modem-control/GPIO pins as GPIO
    // pins, so it is not necessary to do that here.
    sc16is752_write(SC16IS752ChannelA, SC16IS752RegIOIntEna, 0x00);
    sc16is752_write(SC16IS752ChannelA, SC16IS752RegIOState, 0x00);
    sc16is752_write(SC16IS752ChannelA, SC16IS752RegIODir, 0xff);

    for(SC16IS752Channel_t channel = SC16IS752ChannelA; channel <= SC16IS752ChannelB;
        channel += SC16IS752ChannelOffset)
    {
        // Enable RX and TX FIFOs, and set default FIFO trigger levels
        sc16is752_write(channel, SC16IS752RegFCR,
                        SC16IS752RXTrigger8 | SC16IS752TXTrigger8 | SC16IS752_FCR_FIFO_ENABLE);

        // Set default line discipline: 8 bits per word, 1 stop bit, no parity
        sc16is752_write(channel, SC16IS752RegLCR, SC16IS752WordLen8);
    }

    debug_putstr_p("SC16IS752: init done\n");
    return 1;
}


// sc16is752_reset() - execute a software reset of all channels of the SC16IS752 IC, and wait for
// the reset process to complete.  Also reset the RX and TX FIFOs.
//
void sc16is752_reset()
{
    // A software reset is issued by setting the bit SC16IS752_IOCTRL_SRESET in the register
    // SC16IS752RegControl.  There is no need to read this register before setting the SRESET bit
    // as the contents of the register does not survive a software reset.

    // Iterate over the channels in the device
    for(SC16IS752Channel_t channel = SC16IS752ChannelA; channel <= SC16IS752ChannelB;
        channel += SC16IS752ChannelOffset)
    {
        // Initiate the software reset on the channel
        sc16is752_write(channel, SC16IS752RegIOControl, SC16IS752_IOCTRL_SRESET);

        // Wait for the device to reset
        // FIXME - implement a timeout
        while(sc16is752_read(channel, SC16IS752RegIOControl) & SC16IS752_IOCTRL_SRESET)
            ;

        // Reset and enable TX and RX FIFOs
        //
        // Note that there is no way to determine when the TX and RX FIFO reset is complete, as
        // FCR is a write-only register.  The data sheet states that code should "wait at least
        // 2 x T(clk) of XTAL1 before reading or writing data to RHR and THR respectively".  This
        // delay should be unnecessary with a sufficiently fast XTAL1, and is in any case left to
        // the calling code.
        sc16is752_write(channel, SC16IS752RegFCR,
                        SC16IS752_FCR_RESET_TX_FIFO | SC16IS752_FCR_RESET_RX_FIFO |
                        SC16IS752_FCR_FIFO_ENABLE);
    }
}


// sc16is752_set_baud_rate() - set the baud rate on channel <channel> to <baud>.  If <baud> equals
// zero, the baud rate generator will be disabled.  The generator can be re-enabled by calling the
// function again with a non-zero value of <baud>.  Returns non-zero on success and zero on
// failure.  Failure only occurs if the specified baud rate is out of range.
//
uint8_t sc16is752_set_baud_rate(const SC16IS752Channel_t channel, const uint32_t baud)
{
    uint32_t divider;
    uint8_t lcr_val;

    divider = baud ? F_UART / (baud << 4) : 0;

    if(divider > 0xffff)
        return 0;               // Invalid baud rate (too low)

    lcr_val = sc16is752_read(channel, SC16IS752RegLCR);         // Store the current value of LCR

    // Enable access to the baud rate divisor latch
    sc16is752_write(channel, SC16IS752RegLCR, lcr_val | SC16IS752_LCR_DIV_LATCH_EN);

    sc16is752_write(channel, SC16IS752RegDLL, divider & 0xff);  // Write LSB of baud rate divider
    sc16is752_write(channel, SC16IS752RegDLH, divider >> 8);    // Write MSB of baud rate divider

    sc16is752_write(channel, SC16IS752RegLCR, lcr_val);         // Restore the value of LCR

    return 1;
}


// sc16is752_enable_auto_rts() - enable (if <enable> is non-zero), or disable (if <enable> equals
// zero) the use of <channel>'s nRTS signal as a means of RS-485 transceiver flow control.  If this
// feature is enabled, the channel's nRTS signal will be used to indicate whether the channel is in
// transmit or receive mode, and can therefore be used to operate a bidirectional RS-485
// transceiver.  The value of <invert> specifies the sense of the nRTS signal: with <invert> non-
// zero, nRTS will be logic-1 during transmit and logic-0 during receive; if <invert> is equal to
// zero, nRTS will be logic-0 during transmit and logic-1 during receive.
//
void sc16is752_enable_auto_rts(const SC16IS752Channel_t channel, const uint8_t enable,
                               const uint8_t invert)
{
    uint8_t efcr_val;

    efcr_val = sc16is752_read(channel, SC16IS752RegEFCR);

    // Enable or disable RTS inversion, according to the value of <invert>.
    if(invert)
        efcr_val |= SC16IS752_EFCR_RTSINVER;
    else
        efcr_val &= ~SC16IS752_EFCR_RTSINVER;

    // Enable or disable RS-485 RTS direction control, according to the value of <enable>.
    if(enable)
        efcr_val |= SC16IS752_EFCR_RTSCON;
    else
        efcr_val &= ~SC16IS752_EFCR_RTSCON;

    sc16is752_write(channel, SC16IS752RegEFCR, efcr_val);
}


// sc16is752_get_interrupt_mask() - return the contents of the IER (interrupt enable register) for
// the channel specified by <channel>.
//
uint8_t sc16is752_get_interrupt_mask(const SC16IS752Channel_t channel)
{
    return sc16is752_read(channel, SC16IS752RegIER);
}


// sc16is752_set_interrupt_mask() - write <mask> to the IER (interrupt enable register) for the
// channel specified by <channel>.
//
void sc16is752_set_interrupt_mask(const SC16IS752Channel_t channel, const uint8_t mask)
{
    sc16is752_write(channel, SC16IS752RegIER, mask);
}


// sc16is752_get_interrupt_source() - get the interrupt source flags for the channel specified by
// <channel>.  The flags indicate the source of any pending interrupt.
//
SC16IS752IntFlags_t sc16is752_get_interrupt_source(const SC16IS752Channel_t channel)
{
    return (sc16is752_read(channel, SC16IS752RegIIR) & SC16IS752_IIR_PRI_MASK);
}


// sc16is752_tx() - transmit the byte specified by <data> on the channel specified by <channel>.
// Return 0 on success, or 1 if the transmit FIFO is full.
//
uint8_t sc16is752_tx(const SC16IS752Channel_t channel, const uint8_t data)
{
    if(!sc16is752_read(channel, SC16IS752RegTXLVL))
        return 1;       // Insufficient space in FIFO

    sc16is752_write(channel, SC16IS752RegTHR, data);
    return 0;
}


// sc16is752_tx_buf() - write the data pointed to by <buf>, of length <len>, to the FIFO for
// channel <channel>.  Returns non-zero on failure (if there is not sufficient space in the
// transmit FIFO); zero on success.  If data is written to the device, interrupts will be disabled
// around the SPI transaction.
//
uint8_t sc16is752_tx_buf(const SC16IS752Channel_t channel, const void * const buf,
                         const uint8_t len)
{
    const uint8_t *buf_ = (const uint8_t *) buf, *end_ = buf_ + len;

    if(sc16is752_read(channel, SC16IS752RegTXLVL) < len)
        return 1;       // Insufficient space in FIFO

    // Perform a burst write to the peripheral
    interrupt_enable_decrement();
    spi0_slave_select(1);       // Assert SPI_nSS (peripheral select)
    spi0_tx(SC16IS752SPIDirWrite | SC16IS752RegTXLVL | channel);

    while(buf_ != end_)
        spi0_tx(*(buf_++));     // Read and write data

    spi0_slave_select(0);       // Negate SPI_nSS (peripheral select)

    spi0_read();                // } Perform two dummy reads to clear the SPI input buffer
    spi0_read();                // }
    interrupt_enable_increment();

    return 0;
}


// sc16is752_rx() - read the byte at the top of the receive FIFO for the channel specified by
// <channel>.
//
uint8_t sc16is752_rx(const SC16IS752Channel_t channel)
{
    return sc16is752_read(channel, SC16IS752RegRHR);
}


// sc16is752_read() - helper function which performs a SPI read of register <reg>, in channel
// <channel>, and returns the value.
//
static uint8_t sc16is752_read(const SC16IS752Channel_t channel, const SC16IS752Register_t reg)
{
    return sc16is752_spi_op(SC16IS752SPIDirRead, reg, channel, 0);
}


// sc16is752_write() - helper function which writes <data> to register <reg>, in channel <channel>.
//
static void sc16is752_write(const SC16IS752Channel_t channel, const SC16IS752Register_t reg,
                            const uint8_t data)
{
    sc16is752_spi_op(SC16IS752SPIDirWrite, reg, channel, data);
}


// sc16is752_spi_op() - helper function which performs SPI I/O operations.  The argument <dir>
// specifies a direction (register read or register write); <reg> specifies a register; <channel>
// specifies a UART channel, and <data> specifies a value to be written in a write operation.
// The <data> argument is transmitted, but ignored by the device, during read operations.  Returns
// the contents of the SPI read register, which - following a "read" operation - will contain a
// byte sent by the device.
//
static uint8_t sc16is752_spi_op(const SC16IS752SPIDir_t dir, const SC16IS752Channel_t channel,
                                const SC16IS752Register_t reg, const uint8_t data)
{
    uint8_t ret;

    interrupt_enable_decrement();
    spi0_slave_select(1);       // Assert SPI_nSS (peripheral select)
    spi0_tx(dir | reg | channel);
    spi0_tx(data);              // Read and write data
    spi0_slave_select(0);       // Negate SPI_nSS (peripheral select)

    spi0_read();                // Discard the first (dummy) byte received from the peripheral
    ret = spi0_read();          // Read the data returned by the peripheral (if any)
    interrupt_enable_increment();

    return ret;
}


#ifdef DEBUG

// sc16is752_dump_registers() - iterate over the registers in the SC16IS752 IC, and dump their
// output to the debug channel.
//
void sc16is752_dump_registers()
{
    debug_putstr_p("==== SC16IS752 register dump ====\n"
                   "      ChA ChB\n");
    for(SC16IS752Register_t x = 0; x < SC16IS752Reg_End; x += SC16IS752RegOffset)
    {
        const uint8_t data_a = sc16is752_read(SC16IS752ChannelA, x),
                      data_b = sc16is752_read(SC16IS752ChannelB, x);
        debug_printf("R%02x = %02x  %02x\n", x / SC16IS752RegOffset, data_a, data_b);
    }
}

#endif
