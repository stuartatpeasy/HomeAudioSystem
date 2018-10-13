#ifndef DEV_SC16IS752_H_INC
#define DEV_SC16IS752_H_INC
/*
    sc16is752.h - declarations relating to the driver for the SC16IS752 dual SPI/I2C UART IC.

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include <stdint.h>


// SC16IS752Channel_t - enumeration of the two transceiver channels in the SC16IS752.  These values
// are used in the first byte of each SPI transaction.
typedef enum SC16IS752Channel
{
    SC16IS752ChannelA       = (0 << 1),
    SC16IS752ChannelB       = (1 << 1)
} SC16IS752Channel_t;


uint8_t sc16is752_init();
void sc16is752_isr();
void sc16is752_reset();
uint8_t sc16is752_set_baud_rate(const SC16IS752Channel_t channel, const uint32_t baud);
void sc16is752_enable_auto_rts(const SC16IS752Channel_t channel, const uint8_t enable,
                               const uint8_t invert);
uint8_t lc89091_get_error_state();


#ifdef DEBUG
void sc16is752_dump_registers();
#else
#define sc16is752_dump_registers()
#endif

#endif
