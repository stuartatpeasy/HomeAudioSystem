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
            LED1  PB5 -| 6          15 |- PC3  nUART_IRQ
            LED0  PB4 -| 7          14 |- PC2  nSPK_FAULT
       USART_RXD  PB3 -| 8          13 |- PC1  nSPK_SD
       USART_TXD  PB2 -| 9          12 |- PC0  RX_ERR
         I2C_SDA  PB1 -| 10         11 |- PB0  I2C_SCL
                       +---------------+

For BT source module:
    PB4: DE_A
    PB5: DE_B

*/

#include "platform_attinyX16.h"

#define F_CPU       10000000UL                  // Scaled CPU frequency in Hz
#define F_UART      14745600UL                  // SC16IS752 UART crystal frequency in Hz
#define F_EXT_CLK   0UL                         // uC ext clock frequency; 0 if no ext clock is used

// Macro defining the type of module for which we're building firmware
#define MODULE_PA_MONO_TAS5760M                 // Mono power amplifier based on TAS5760M
//#define MODULE_BT_SOURCE_V1                   // Bluetooth audio source v1
//#define MODULE_STEREO_SOURCE_V1

//
// Pin definitions
//

// Port A
#define PIN_VCC_AMP                 GPIOA(7)    // [AI] Amplifier supply voltage (scaled)
#define PIN_VCC_IN                  GPIOA(6)    // [AI] System supply voltage (scaled)

// Port B
#define PIN_LED1                    GPIOB(5)    // [O]
#define PIN_LED0                    GPIOB(4)    //
#define PIN_I2C_SDA                 GPIOB(1)    // [I/O] I2C data
#define PIN_I2C_SCL                 GPIOB(0)    // [I/O] I2C clock

// Port C
#define PIN_nUART_IRQ               GPIOC(3)    // [I] SC16IS752 UART interrupt request output
#define PIN_nSPK_FAULT              GPIOC(2)    // [I] TAS5760M speaker fault output
#define PIN_nSPK_SD                 GPIOC(1)    // [O] TAS5760M speaker shutdown input
#define PIN_RX_ERR                  GPIOC(0)    // [I] LC89091 error output


#endif
