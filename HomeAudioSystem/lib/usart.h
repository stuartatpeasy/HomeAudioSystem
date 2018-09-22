#ifndef LIB_USART_H_INC
#define LIB_USART_H_INC
/*
    usart.h: USART helper function declarations

    Stuart Wallace <stuartw@atom.net>, July 2018.
*/

#include <avr/io.h>
#include "types.h"


#define BAUDREG_VAL_MIN         (0x004a)            // Minimum allowable value in the BAUD register
#define BPW_MIN                 (5)                 // Minimum allowable bits-per-word
#define BPW_MAX                 (8)                 // Maximum allowable bits-per-word


// USARTParity_t - parity modes
//
typedef enum USARTParity
{
    USARTParityNone     = 0,            // USART uses no parity
    USARTParityEven     = 2,            // USART uses even parity
    USARTParityOdd      = 3             // USART uses odd parity
} USARTParity_t;


// Flags used by usart0_enable() to determine whether to enable or disable the receiver and
// transmitter
#define USART_ENABLE_RX         USART_RXEN_bm       // Receiver-enable flag
#define USART_ENABLE_TX         USART_TXEN_bm       // Transmitter-enable flag


// usart0_flush_dre() - macro to busy-wait until the USART0 transmit data register is empty.
//
#define usart0_flush_dre()                                  \
            do                                              \
            {                                               \
                while(!(USART0_STATUS & USART_DREIF_bm))    \
                    ;                                       \
            } while(0)


// usart0_flush_tx() - macro to busy-wait until all data has been flushed (i.e. sent) from the
// USART0 transmit buffer.
//
#define usart0_flush_tx()                                   \
            do                                              \
            {                                               \
                while(!(USART0_STATUS & USART_TXCIF_bm))    \
                    ;                                       \
            } while(0)


// usart0_tx(data) - wait for the USART0 transmit data register to become empty, then coerce the
// value in <data> to a uint_fast8_t and transmit it over USART0.
//
#define usart0_tx(data)                                     \
            do                                              \
            {                                               \
                usart0_flush_dre();                         \
                USART0_TXDATAL = (uint_fast8_t) data;       \
            } while (0)


void usart0_configure_io(const Pinset_t pinset);
uint8_t usart0_configure_port(const uint32_t baud, const uint8_t bpw, const USARTParity_t parity);
void usart0_enable(const uint8_t enable);
void usart0_puthex_byte(const uint8_t data);
void usart0_puthex_word(const uint16_t data);
void usart0_puts(const char *str);
void usart0_puts_p(const char *str);
uint8_t usart0_set_baud_rate(const uint32_t baud);
uint32_t usart0_get_baud_rate();

#endif
