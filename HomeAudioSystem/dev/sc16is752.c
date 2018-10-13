/*
    sc16is752.c - definitions relating to the driver for the SC16IS752 dual SPI/I2C UART IC.

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "sc16is752.h"
#include "../platform.h"        // for F_UART
#include "../lib/debug.h"
#include "../lib/spi.h"


// SC16IS752Register_t - enumeration of the registers in the SC16IS752.  Some of these registers
// share an address.  In such cases, bits in other registers determine which register is actually
// accessed.
//
typedef enum SC16IS752Register
{
    SC16IS752RegRHR_THR     = (0x00 << 3),  // Receive (RHR) / transmit (THR) holding register
    SC16IS752RegDLL         = (0x00 << 3),  // Baud rate divider - least-significant byte
    SC16IS752RegIER         = (0x01 << 3),  // Interrupt enable register
    SC16IS752RegDLH         = (0x01 << 3),  // Baud rate divider - most-significant byte
    SC16IS752RegFCR_IIR     = (0x02 << 3),  // FIFO control reg (write) / interrupt ID reg (read)
    SC16IS752RegEFR         = (0x02 << 3),  // Enhanced features register
    SC16IS752RegLCR         = (0x03 << 3),  // Line control register
    SC16IS752RegMCR         = (0x04 << 3),  // Modem control register
    SC16IS752RegXON1        = (0x04 << 3),  // XON1 word
    SC16IS752RegLSR         = (0x05 << 3),  // Line status register
    SC16IS752RegXON2        = (0x05 << 3),  // XON2 word
    SC16IS752RegMSR_TCR     = (0x06 << 3),  // Modem status / transmission control register
    SC16IS752RegXOFF1       = (0x06 << 3),  // XOFF1 word
    SC16IS752RegSPR_TLR     = (0x07 << 3),  // Scratch pad register / trigger level register
    SC16IS752RegXOFF2       = (0x07 << 3),  // XOFF2 word
    SC16IS752RegTXLVL       = (0x08 << 3),  // Transmitter FIFO level register
    SC16IS752RegRXLVL       = (0x09 << 3),  // Receiver FIFO level register
    SC16IS752RegIODir       = (0x0a << 3),  // Programmable IO pins direction register
    SC16IS752RegIOState     = (0x0b << 3),  // Programmable IO pins state register
    SC16IS752RegIOIntEna    = (0x0c << 3),  // Programmable IO pins interrupt enable register
    SC16IS752RegIOControl   = (0x0e << 3),  // Programmable IO pins control register
    SC16IS752RegEFCR        = (0x0f << 3),  // Extra features control register
    SC16IS752Reg_End                        // End-of-list sentinel
} SC16IS752Register_t;

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

    for(SC16IS752Channel_t channel = SC16IS752ChannelA; channel <= SC16IS752ChannelB; ++channel)
    {
        // Enable RX and TX FIFOs, and set default FIFO trigger levels
        sc16is752_write(channel, SC16IS752RegFCR_IIR,
                        SC16IS752RXTrigger8 | SC16IS752_FCR_FIFO_ENABLE);

        // Set default line discipline: 8 bits per word, 1 stop bit, no parity
        sc16is752_write(channel, SC16IS752RegLCR, SC16IS752WordLen8);
    }

    debug_putstr_p("SC16IS752: init done\n");
    return 1;
}


// sc16is752_isr() - interrupt handler.  This function should be called by a generic pin-state
// interrupt service routine if the nUART_IRQ pin is asserted.  It therefore executes in interrupt
// context.
//
void sc16is752_isr()
{

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
    for(SC16IS752Channel_t channel = SC16IS752ChannelA; channel <= SC16IS752ChannelB; ++channel)
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
        sc16is752_write(channel, SC16IS752RegFCR_IIR,
                        SC16IS752_FCR_RESET_TX_FIFO | SC16IS752_FCR_RESET_RX_FIFO);
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
    spi0_slave_select(1);       // Assert SPI_nSS (slave select)
    spi0_tx(dir | reg | channel);
    spi0_tx(data);              // Read and write data
    spi0_slave_select(0);       // Negate SPI_nSS (slave select)

    return spi0_read();         // Return the contents of the SPI read buffer
}

#ifdef DEBUG

// sc16is752_dump_registers() - iterate over the registers in the SC16IS752 IC, and dump their
// output to the debug channel.
//
void sc16is752_dump_registers()
{
    debug_putstr_p("==== SC16IS752 register dump ====\n");
    for(SC16IS752Register_t x = 0; x < SC16IS752Reg_End; x += 8)
    {
        const uint8_t data = sc16is752_read(SC16IS752ChannelA, x);
        debug_printf("R%02x = %02x\n", x >> 3, data);
    }
}

#endif
