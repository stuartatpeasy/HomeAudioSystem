#ifndef PLATFORM_ATTINYX14_H_INC
#define PLATFORM_ATTINYX14_H_INC
/*
    platform_attinyx16.h - pin declarations for the ATTiny416/816 uCs.

    Stuart Wallace <stuartw@atom.net>, September 2018.

    Note that these declarations make use of the GPIO*() macros defined in lib/gpio.h.
*/


// Port A
#define PIN_SPI_nSS_DEFAULT         GPIOA(4)    // [O]  SPI slave select (default pin-set)
#define PIN_SPI_SCK_DEFAULT         GPIOA(3)    // [O]  SPI clock (default pin-set)
#define PIN_SPI_MISO_DEFAULT        GPIOA(2)    // [I]  SPI MISO (default pin-set)
#define PIN_USART_RXD_ALT           GPIOA(2)    // [I]  USART RXD (alternate pin-set)
#define PIN_SPI_MOSI_DEFAULT        GPIOA(1)    // [O]  SPI MOSI (default pin-set)
#define PIN_USART_TXD_ALT           GPIOA(1)    // [O]  USART TXD (alternate pin-set)

// Port B
#define PIN_USART_RXD_DEFAULT       GPIOB(3)    // [I]  USART RXD (default pin-set)
#define PIN_USART_TXD_DEFAULT       GPIOB(2)    // [O]  USART TXD (default pin-set)
#define PIN_TWI_SDA                 GPIOB(1)    // [IO] TWI (I2C) SDA (data)
#define PIN_TWI_SCL                 GPIOB(0)    // [O]  TWI (I2C) SCL (clock)

#endif // PLATFORM_ATTINYX14_H_INC
