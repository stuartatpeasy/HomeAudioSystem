/*
    gpio.c - declarations for helper functions relating to GPIO ports

    Stuart Wallace <stuartw@atom.net>, July 2018.
*/

#include "gpio.h"
#include <avr/pgmspace.h>


// Array of GPIO-manipulation registers
static volatile uint8_t * const reg_map[] PROGMEM =
{
#if defined(WITH_ATTINY816) || defined(WITH_ATTINY814)
    &PORTA_DIRSET, &PORTA_DIRCLR, &PORTA_OUTSET, &PORTA_OUTCLR, &PORTA_IN, &PORTA_DIR, &PORTA_PIN0CTRL
  , &PORTB_DIRSET, &PORTB_DIRCLR, &PORTB_OUTSET, &PORTB_OUTCLR, &PORTB_IN, &PORTB_DIR, &PORTB_PIN0CTRL
#if defined(WITH_ATTINY816)
  , &PORTC_DIRSET, &PORTC_DIRCLR, &PORTC_OUTSET, &PORTC_OUTCLR, &PORTC_IN, &PORTC_DIR, &PORTC_PIN0CTRL
#endif // WITH_ATTINY816
#endif // WITH_ATTINY816 || WITH_ATTINY814
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


// gpio_get_sense() - get the input/sense configuration for a pin.
//
GPIOSense_t gpio_get_sense(const GPIOPin_t pin)
{
    return *(register_address(pin, GPIOActionPinCtrl) + pin.pin) & PORT_ISC_gm;
}


// gpio_set_sense() - set the input/sense configuration for a pin.
//
void gpio_set_sense(const GPIOPin_t pin, const GPIOSense_t sense)
{
    volatile uint8_t * const reg = register_address(pin, GPIOActionPinCtrl) + pin.pin;
    *reg = (*reg & ~PORT_ISC_gm) | sense;
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


// gpio_get_pullup() - get the enabled/disabled state of the internal pullup resistor on the
// specified pin.  Returns non-zero if the pullup is enabled; zero if it is disabled.
//
uint8_t gpio_get_pullup(const GPIOPin_t pin)
{
    return *(register_address(pin, GPIOActionPinCtrl) + pin.pin) & PORT_PULLUPEN_bm;
}


// gpio_set_pullup() - enable or disable the internal pullup resistor on the specified pin.
//
void gpio_set_pullup(const GPIOPin_t pin, const uint8_t enable)
{
    volatile uint8_t * const reg = register_address(pin, GPIOActionPinCtrl) + pin.pin;

    if(enable)
        *reg |= PORT_PULLUPEN_bm;
    else
        *reg &= ~PORT_PULLUPEN_bm;
}


// gpio_get_invert() - get the state of the invert flag for the specified pin.  Returns non-zero if
// the inverter is enabled for the pin; zero if the inverter is disabled.
//
uint8_t gpio_get_invert(const GPIOPin_t pin)
{
    return *(register_address(pin, GPIOActionPinCtrl) + pin.pin) & PORT_INVEN_bm;
}


// gpio_set_invert() - set the state of the invert flag for the specified pin, thereby enabling or
// disabling the inverter attached to the pin.
//
void gpio_set_invert(const GPIOPin_t pin, const uint8_t enable)
{
    volatile uint8_t * const reg = register_address(pin, GPIOActionPinCtrl) + pin.pin;

    if(enable)
        *reg |= PORT_INVEN_bm;
    else
        *reg &= ~PORT_INVEN_bm;
}
