#ifndef PLATFORM_H_INC
#define PLATFORM_H_INC
/*
    platform.h: declarations relevant to both main.c and other modules

    Stuart Wallace <stuartw@atom.net>, July 2018.

    Note that these declarations make use of the GPIO*() macros defined in lib/gpio.h.


                           ATtiny816
                       +---------------+
                  VDD -| 1          20 |- GND
         SPI_nSS  PA4 -| 2          19 |- PA3  SPI_SCK
                  PA5 -| 3          18 |- PA2  SPI_MISO
          VCC_IN  PA6 -| 4          17 |- PA1  SPI_MOSI
         VCC_AMP  PA7 -| 5          16 |- PA0  UPDI
                  PB5 -| 6          15 |- PC3
                  PB4 -| 7          14 |- PC2  nSPK_FAULT
       USART_RXD  PB3 -| 8          13 |- PC1  nSPK_SD
       USART_TXD  PB2 -| 9          12 |- PC0  RX_ERR
         I2C_SDA  PB1 -| 10         11 |- PB0  I2C_SCL
                       +---------------+

    USART_TXD/_RXD will be used for:
    - debug interface on PA module
    - BT control interface on BT module
    * MOVE VCC_AMP      A3->A7
    * MOVE nSPK_SD      A5->C0
    * MOVE nSPK_FAULT   A6->C1
    * MOVE RX_ERR       B5->C2
*/

#include "platform_attinyX16.h"

#define F_CPU       20000000UL

//
// Pin definitions
//

// Port A
#define PIN_VCC_AMP                 GPIOA(7)    // [AI] Amplifier scaled supply voltage input

// Port B
#define PIN_I2C_SDA                 GPIOB(1)    // [I/O] I2C data
#define PIN_I2C_SCL                 GPIOB(0)    // [I/O] I2C clock

// Port C
#define PIN_nSPK_FAULT              GPIOC(2)    // [I] TAS5760M speaker fault output
#define PIN_nSPK_SD                 GPIOC(1)    // [O] TAS5760M speaker shutdown input
#define PIN_RX_ERR                  GPIOC(0)    // [I] LC89091 error output


#endif
