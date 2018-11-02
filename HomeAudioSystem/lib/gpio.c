/*
    gpio.c - declarations for helper functions relating to GPIO ports

    Stuart Wallace <stuartw@atom.net>, July 2018.
*/

#include "gpio.h"
#include <avr/pgmspace.h>


// Array of GPIO-manipulation registers
static volatile uint8_t * const reg_map[] PROGMEM =
{
#if defined(WITH_ATTINY816)
    &PORTA_DIRSET, &PORTA_DIRCLR, &PORTA_OUTSET, &PORTA_OUTCLR, &PORTA_IN, &PORTA_DIR, &PORTA_PIN0CTRL,
    &PORTB_DIRSET, &PORTB_DIRCLR, &PORTB_OUTSET, &PORTB_OUTCLR, &PORTB_IN, &PORTB_DIR, &PORTB_PIN0CTRL,
    &PORTC_DIRSET, &PORTC_DIRCLR, &PORTC_OUTSET, &PORTC_OUTCLR, &PORTC_IN, &PORTC_DIR, &PORTC_PIN0CTRL
#elif defined(WITH_ATTINY814)
    &PORTA_DIRSET, &PORTA_DIRCLR, &PORTA_OUTSET, &PORTA_OUTCLR, &PORTA_IN, &PORTA_DIR, &PORTA_PIN0CTRL,
    &PORTB_DIRSET, &PORTB_DIRCLR, &PORTB_OUTSET, &PORTB_OUTCLR, &PORTB_IN, &PORTB_DIR, &PORTB_PIN0CTRL
#endif
};


// register_address() - get the address of the GPIO register corresponding to the action and port
// combination specified by <action> and <pin>.
//
static inline volatile uint8_t *register_address(const GPIOPin_t pin, const GPIOAction_t action)
{
    return (volatile uint8_t *) pgm_read_word(reg_map + (GPIOAction_end * pin.port) + action);
}


// gpio_action_write() - perform the action specified by <action> on the port+pin combination
// specified in <pin>.  This function looks up the appropriate IO register and writes a 1-bit to
// the register bit corresponding to the specified port pin.
//
void gpio_action_write(const GPIOPin_t pin, const GPIOAction_t action)
{
    *register_address(pin, action) = 1 << pin.pin;
}


// gpio_action_read() - calculate the address of the GPIO-input register appropriate to the action
// specified by <action> and the port specified in <pin>, read that register, and return the bit
// corresponding to the pin specified in <pin>.
//
uint8_t gpio_action_read(const GPIOPin_t pin, const GPIOAction_t action)
{
    return *register_address(pin, action) & (1 << pin.pin);
}


// gpio_wait_high() - busy-wait until the specified pin reads as logic 1.
//
void gpio_wait_high(const GPIOPin_t pin)
{
    volatile uint8_t * const reg = register_address(pin, GPIOActionRead);

    while(!(*reg & (1 << pin.pin)))
        ;
}


// gpio_wait_low() - busy-wait until the specified pin reads as logic 0.
//
void gpio_wait_low(const GPIOPin_t pin)
{
    volatile uint8_t * const reg = register_address(pin, GPIOActionRead);

    while(*reg & (1 << pin.pin))
        ;
}


// gpio_set_sense() - set the input/sense configuration for a pin
//
void gpio_set_sense(const GPIOPin_t pin, const GPIOSense_t sense)
{
    *(register_address(pin, GPIOActionPinCtrl) + pin.pin) = sense;
}


// gpio_set_level() - set pin <pin>, which is assumed to be an output, to logic 1 if <level> is
// non-zero, or logic 0 if <level> equals zero.
//
void gpio_set_level(const GPIOPin_t pin, const uint8_t level)
{
    if(level)
        gpio_set(pin);
    else
        gpio_clear(pin);
}
