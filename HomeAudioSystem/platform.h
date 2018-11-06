#ifndef PLATFORM_H_INC
#define PLATFORM_H_INC
/*
    platform.h: declarations relevant to both main.c and other modules

    Stuart Wallace <stuartw@atom.net>, July 2018.

    Note that the pinout declarations in this file make use of the GPIO*() macros defined in
    lib/gpio.h.
*/

// Note that a macro, defining the module type for which we're building firmware, must be defined
// somewhere.  This definition currently resides in the project configuration.  Before build, a
// specific configuration is selected: e.g. "PA_MONO_TAS5760M_Debug", for a debug build of firmware
// for a TAS5760M-based mono PA module.  Macros to configure this build are then defined under
// Project -> Properties... -> AVR/GNU C Compiler -> Symbols.


//
// Definitions common to all module types
//

// Currently all modules make use of the SC16IS752 to implement the control interface
#define WITH_SC16IS752
#define F_UART      14745600UL                  // SC16IS752 UART crystal frequency in Hz
#define F_CPU       10000000UL                  // Scaled CPU frequency in Hz
#define F_EXT_CLK   0UL                         // uC ext clock frequency; 0 if no ext clock is used

//
// Per-module pin-out, compilation and configuration directives
//

#ifdef MODULE_PA_MONO_TAS5760M
/*
                           ATtiny816
                       +---------------+
                  VDD -| 1          20 |- GND
         SPI_nSS  PA4 -| 2          19 |- PA3  SPI_SCK
       VCC_IN_LV  PA5 -| 3          18 |- PA2  SPI_MISO
       VCC_IN_HV  PA6 -| 4          17 |- PA1  SPI_MOSI
         VCC_AMP  PA7 -| 5          16 |- PA0  UPDI
                  PB5 -| 6          15 |- PC3  nUART_IRQ
                  PB4 -| 7          14 |- PC2  nSPK_FAULT
                  PB3 -| 8          13 |- PC1  nSPK_SD
       USART_TXD  PB2 -| 9          12 |- PC0  RX_ERR
         I2C_SDA  PB1 -| 10         11 |- PB0  I2C_SCL
                       +---------------+
*/

#define WITH_LC89091JA
#define WITH_PCA9632
#define WITH_TAS5760M
#define WITH_ATTINY816

// Pin definitions
//
// Port A
#define PIN_VCC_AMP                 GPIOA(7)    // [AI] Amplifier supply voltage (scaled)
#define PIN_VCC_IN                  GPIOA(6)    // [AI] System supply voltage (scaled)

// Port B
#define PIN_I2C_SDA                 GPIOB(1)    // [IO] I2C data
#define PIN_I2C_SCL                 GPIOB(0)    // [IO] I2C clock

// Port C
#define PIN_nUART_IRQ               GPIOC(3)    // [I]  SC16IS752 UART interrupt request output
#define PIN_nSPK_FAULT              GPIOC(2)    // [I]  TAS5760M speaker fault output
#define PIN_nSPK_SD                 GPIOC(1)    // [O]  TAS5760M speaker shutdown input
#define PIN_RX_ERR                  GPIOC(0)    // [I]  LC89091 error output

#elif defined(MODULE_BT_SOURCE_V1)
/*
                           ATtiny814
                       +---------------+
                  VDD -| 1          14 |- GND
        UART_nSS  PA4 -| 2          13 |- PA3  UART_SCLK
       UART_nIRQ  PA5 -| 3          12 |- PA2  UART_MISO
         BT_DE_B  PA6 -| 4          11 |- PA1  UART_MOSI
         BT_DE_A  PA7 -| 5          10 |- PA0  UPDI
      BT_UART_TX  PB3 -| 6           9 |- PB0  I2C_SCL
      BT_UART_RX  PB2 -| 7           8 |- PB1  I2C_SDA
                       +---------------+
*/

#define WITH_PCA9632
#define WITH_RN52
#define WITH_ATTINY814

// Pin definitions
//
// Port A
#define PIN_BT_DE_A                 GPIOA(7)    // [O]  Audio data driver enable - channel B
#define PIN_BT_DE_B                 GPIOA(6)    // [O]  Audio data driver enable - channel A
#define PIN_UART_nIRQ               GPIOA(5)    // [I]  IRQ input from UART
#define PIN_UART_nSS                GPIOA(4)    // [O]  UART SPI slave select
#define PIN_UART_SCLK               GPIOA(3)    // [O]  UART SPI serial clock
#define PIN_UART_MISO               GPIOA(2)    // [I]  UART SPI master-in, slave-out
#define PIN_UART_MOSI               GPIOA(1)    // [I]  UART SPI master-out, slave-in

// Port B
#define BT_UART_TX                  GPIOB(3)    // [I]  Bluetooth module UART TX (input to uC)
#define BT_UART_RX                  GPIOB(2)    // [O]  Bluetooth module UART RX (output from uC)
#define I2C_SDA                     GPIOB(1)    // [IO] I2C (aka TWI) data
#define I2C_SCL                     GPIOB(0)    // [O]  I2C (aka TWI) clock

#else
#error "No module type specified - check compiler directives"
#endif      // Module-specific directives
#endif      // PLATFORM_H_INC

#if defined(WITH_ATTINY816)
#include "platform_attinyX16.h"
#elif defined(WITH_ATTINY814)
#include "platform_attinyX14.h"
#endif
