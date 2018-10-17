#ifndef UTIL_IRQ_H_INC
#define UTIL_IRQ_H_INC
/*
    irq.h: helper functions to support nested enabling/disabling of interrupts

    Stuart Wallace <stuartw@atom.net>, October 2018.
*/

#include <avr/interrupt.h>

extern int8_t g_irq_state;


inline void interrupt_enable_increment()
{
    if(++g_irq_state > 0)
        sei();
};


inline void interrupt_enable_decrement()
{
    if(--g_irq_state <= 0)
        cli();
};

#endif
