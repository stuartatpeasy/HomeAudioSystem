/*
    spi.c - definitions related to SPI helper functions

    Stuart Wallace <stuartw@atom.net>, July 2018.
*/

#include "spi.h"
#include "gpio.h"
#include "platform.h"


static GPIOPin_t SPI_nSS;


// spi0_configure_master() - configure the SPI0 peripheral as a master, and set its clock divider
// according to the value in <div>.  This function unconditionally sets certain options in the SPI
// peripheral: byte-order is hard-wired to MSB-first, 2x-clock mode is disabled, buffered mode is
// enabled, and nSS is ignored.
//
void spi0_configure_master(const Pinset_t pinset, const SPIClkDiv_t div)
{
    if(pinset == PinsetDefault)
    {
        PORTMUX_CTRLB &= ~PORTMUX_SPI0_ALTERNATE_gc;
        SPI_nSS = PIN_SPI_nSS_DEFAULT;
    }
    else
    {
        PORTMUX_CTRLB |= PORTMUX_SPI0_ALTERNATE_gc;
        SPI_nSS = PIN_SPI_nSS_ALT;
    }

    // Select SPI master mode and set prescaler.  Also: accept the default data order (MSB first),
    // and disable 2x-clock.  Do this without affecting the "enabled" state of the peripheral.
    SPI0_CTRLA = (SPI0_CTRLA & SPI_ENABLE_bm) | SPI_MASTER_bm | div;

    // Enable buffered mode, select SPI Mode 0, and instruct the peripheral to ignore the value at
    // the nSS pin.
    SPI0_CTRLB = SPI_BUFEN_bm | SPI_SSD_bm;
}


// spi0_port_activate() - activate (if <activate> is non-zero) or deactivate (if <activate> equals
// zero) the SPI port.  Activation entails configuring as outputs the pins associated with SPI
// output signals.  Deactivation entails configuring all SPI pins as inputs.  In both cases,
// "resting" (i.e. inactive and safe) logic levels are set on the port pins before each pin's
// direction is set.
//
void spi0_port_activate(const uint8_t activate)
{
    GPIOPin_t mosi, miso, sck;

    if(GPIOPIN_EQUAL(SPI_nSS, PIN_SPI_nSS_DEFAULT))
    {
        // Default pin-set
        mosi = PIN_SPI_MOSI_DEFAULT;
        miso = PIN_SPI_MISO_DEFAULT;
        sck = PIN_SPI_SCK_DEFAULT;
    }
    else
    {
        // Alternate pin-set
        mosi = PIN_SPI_MOSI_ALT;
        miso = PIN_SPI_MISO_ALT;
        sck = PIN_SPI_SCK_ALT;
    }

    // Set "resting" logic levels on the SPI port output pins
    gpio_set(SPI_nSS);
    gpio_clear(mosi);
    gpio_clear(sck);

    gpio_make_input(miso);          // MISO is always an input

    if(activate)
    {
        // Set pin directions for an active port: MOSI, SCK, nSS = outputs
        gpio_make_output(mosi);
        gpio_make_output(sck);
        gpio_make_output(SPI_nSS);
    }
    else
    {
        // Set pin directions for an inactive (sleeping) port: MOSI, SCK, nSS = inputs
        gpio_make_input(mosi);
        gpio_make_input(sck);
        gpio_make_input(SPI_nSS);
    }
}


// spi0_enable() - enable or disable the SPI0 peripheral.  If <enable> is non-zero, the peripheral
// is enabled; otherwise it is disabled.
//
void spi0_enable(const uint8_t enable)
{
    if(enable)
        SPI0_CTRLA |= SPI_ENABLE_bm;
    else
        SPI0_CTRLA &= ~SPI_ENABLE_bm;
}


// spi0_slave_select() - if <select> is non-zero, assert (i.e. set to logic 0) the SPI nSS line;
// otherwise negate (i.e. set to logic 1) the SPI nSS line.
//
void spi0_slave_select(const uint8_t select)
{
    if(select)
        gpio_clear(SPI_nSS);
    else
        gpio_set(SPI_nSS);
}
