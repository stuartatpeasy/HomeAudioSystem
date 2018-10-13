/*
    usart.c: USART helper function definitions

    Stuart Wallace <stuartw@atom.net>, July 2018.
*/

#include "usart.h"
#include "clk.h"
#include "gpio.h"
#include <avr/pgmspace.h>


// Map of hex values to ASCII
static const uint8_t hex_map[16] PROGMEM = {'0', '1', '2', '3', '4', '5', '6', '7',
                                            '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};


// usart0_configure_io() - configure the IO pins for the USART module.  The <pinset> argument
// specifies whether to use the default (if <pinset> == USART0_PINSET_DEFAULT) or alternative (if
// <pinset> == USART0_PINSET_ALTERNATE) pins.  The appropriate TXD pin is set to logic 1 and made
// an output; the RXD pin is made an input.  Interrupts must be disabled when this function is
// called.
//
void usart0_configure_io(const Pinset_t pinset)
{
    GPIOPin_t rxd, txd;

    if(pinset == PinsetDefault)
    {
        PORTMUX_CTRLB &= ~PORTMUX_USART0_ALTERNATE_gc;  // Select the default pin-set for USART0
        rxd = PIN_USART_RXD_DEFAULT;
        txd = PIN_USART_TXD_DEFAULT;

    }
    else
    {
        PORTMUX_CTRLB |= PORTMUX_USART0_ALTERNATE_gc;   // Select the alternate pin-set for USART0
        rxd = PIN_USART_RXD_ALT;
        txd = PIN_USART_TXD_ALT;
    }

    gpio_set(txd);                  // Set the USART TXD pin to logic 1
    gpio_make_input(rxd);           // Make the USART RXD pin an input
    gpio_make_output(txd);          // Make the USART TXD pin an output
}


uint8_t usart0_configure_port(const uint32_t baud, const uint8_t bpw, const USARTParity_t parity)
{
    if((bpw < BPW_MIN) || (bpw > BPW_MAX))
        return 0;

//  FIXME - complete this fn
//    USART0_CTRLC

    return 1;
}


// usart0_enable() - selectively enable or disable the USART0 receiver and transmitter based on the
// value of the <enable> argument.  <enable> is a bitmap of USART_ENABLE_RX and USART_ENABLE_TX.
//
void usart0_enable(const uint8_t enable)
{
    USART0_CTRLB = (USART0_CTRLB & ~(USART_RXEN_bm | USART_TXEN_bm)) | enable;
}


// usart0_set_baud_rate() - attempt to set the USART0 baud rate to the value specified by <baud>.
// Return non-zero on success, or zero if the specified baud rate is out of range or the peripheral
// clock frequency cannot be determined.
//
uint8_t usart0_set_baud_rate(const uint32_t baud)
{
    const uint32_t pclk_freq = pclk_get_freq();
    uint16_t baudreg_val;

    if(!pclk_freq || !baud)
        return 0;

    // Obtain the number of clocks per sample on the RXD pin.  This will be 16 in the "normal"
    // modes, and 8 in the "double clock" mode.  TODO: check that this assumption is valid for the
    // auto-baud modes (USART_RXMODE_GENAUTO and USART_RXMODE_LINAUTO).
    const uint8_t nsamples = (USART0_CTRLB & USART_RXMODE_gm) == USART_RXMODE_CLK2X_gc ? 8 : 16;

    // FIXME - verify that this won't overflow
    baudreg_val = (64 / nsamples) * pclk_freq / baud;

    if(baudreg_val < BAUDREG_VAL_MIN)
        return 0;

    USART0_BAUD = baudreg_val;

    return 1;
}


// usart0_get_baud_rate() - get USART0's current baud rate.  Return 0 if the peripheral clock
// frequency cannot be determined, or if the value in the BAUD register is 0.
//
uint32_t usart0_get_baud_rate()
{
    const uint16_t baud_reg_val = USART0_BAUD;
    const uint32_t pclk_freq = pclk_get_freq();

    // Obtain the number of clocks per sample on the RXD pin.  This will be 16 in the "normal"
    // modes, and 8 in the "double clock" mode.  TODO: check that this assumption is valid for the
    // auto-baud modes (USART_RXMODE_GENAUTO and USART_RXMODE_LINAUTO).
    const uint8_t nsamples = (USART0_CTRLB & USART_RXMODE_gm) == USART_RXMODE_CLK2X_gc ? 8 : 16;

    if(!baud_reg_val || !pclk_freq)
        return 0;

    // FIXME - verify that this won't overflow
    return (64 / nsamples) * pclk_freq / baud_reg_val;
}


// usart0_puthex_byte() - write the byte value in <data> to USART0 as a two-character hex string.
//
void usart0_puthex_byte(const uint8_t data)
{
    usart0_tx(pgm_read_byte(hex_map + (data >> 4)));
    usart0_tx(pgm_read_byte(hex_map + (data & 0xf)));
}


// usart0_puthex_word() - write the word (16-bit) value in <data> to USART0 as a four-character hex
// string.
//
void usart0_puthex_word(const uint16_t data)
{
    usart0_puthex_byte(data >> 8);
    usart0_puthex_byte(data & 0xff);
}


// usart0_puts() - write the string <str> to USART0.
//
void usart0_puts(const char *str)
{
    while(*str)
        usart0_tx(*str++);
}


// usart0_puts_p - write the string <str>, which is situated in program memory, to USART0.
//
void usart0_puts_p(const char *str)
{
    uint8_t c;
    while((c = pgm_read_byte(str++)) != '\0')
        usart0_tx(c);
}

