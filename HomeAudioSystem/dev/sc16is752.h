#ifndef DEV_SC16IS752_H_INC
#define DEV_SC16IS752_H_INC
/*
    sc16is752.h - declarations relating to the driver for the SC16IS752 dual SPI/I2C UART IC.

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "platform.h"
#include <stdint.h>

#ifdef WITH_SC16IS752

// SC16IS752Channel_t - enumeration of the two transceiver channels in the SC16IS752.  These values
// are used in the first byte of each SPI transaction.
typedef enum SC16IS752Channel
{
    SC16IS752ChannelA       = (0 << 1),
    SC16IS752ChannelB       = (1 << 1)
} SC16IS752Channel_t;

// Macro defining the size of the "stride" between channels, to simplify iterating over channels.
#define SC16IS752ChannelOffset      (SC16IS752ChannelB - SC16IS752ChannelA)

// Interrupt flags
#define SC16IS752_INT_nCTS              (0x80)
#define SC16IS752_INT_nRTS              (0x40)
#define SC16IS752_INT_XOFF              (0x20)
#define SC16IS752_INT_SLEEP_MODE        (0x10)
#define SC16IS752_INT_MODEM_STATUS      (0x08)
#define SC16IS752_INT_RX_LINE_STATUS    (0x04)
#define SC16IS752_INT_THR_EMPTY         (0x02)
#define SC16IS752_INT_RX_DATA_AVAILABLE (0x01)

// Wrapper type for device interrupt flags
typedef uint8_t SC16IS752IntFlags_t;

// Macros that can be used to identify an interrupt, given an SC16IS752IntFlags_t in <x>
#define SC16IS752IntRXLineStatus(x)     ((x) == 0x06)
#define SC16IS752IntRXTimeout(x)        ((x) == 0x0c)
#define SC16IS752IntRXDataReady(x)      ((x) == 0x04)
#define SC16IS752IntTXDataReady(x)      ((x) == 0x02)
#define SC16IS752IntModemStatus(x)      ((x) == 0x00)
#define SC16IS752IntIOPins(x)           ((x) == 0x30)
#define SC16IS752IntXoff(x)             ((x) == 0x10)
#define SC16IS752IntRTSCTS(x)           ((x) == 0x20)

uint8_t sc16is752_init();
void sc16is752_reset();
uint8_t sc16is752_set_baud_rate(const SC16IS752Channel_t channel, const uint32_t baud);
void sc16is752_enable_auto_rts(const SC16IS752Channel_t channel, const uint8_t enable,
                               const uint8_t invert);
uint8_t sc16is752_tx(const SC16IS752Channel_t channel, const uint8_t data);
uint8_t sc16is752_tx_buf(const SC16IS752Channel_t channel, const void * const buf,
                         const uint8_t len);
uint8_t sc16is752_rx(const SC16IS752Channel_t channel);
uint8_t sc16is752_get_interrupt_mask(const SC16IS752Channel_t channel);
void sc16is752_set_interrupt_mask(const SC16IS752Channel_t channel, const uint8_t mask);
SC16IS752IntFlags_t sc16is752_get_interrupt_source(const SC16IS752Channel_t channel);


#ifdef DEBUG
void sc16is752_dump_registers();
#else
#define sc16is752_dump_registers()
#endif

#endif // WITH_SC16IS752
#endif // DEV_SC16IS752_H_INC
