#ifndef DEV_TAS5760M_H_INC
#define DEV_TAS5760M_H_INC
/*
    tas5760m.h - declarations relating to the driver for the TAS5760M I2S-input class-D audio
    amplifier IC.

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include <stdint.h>



// TAS5760M error constants, from the Fault Configuration / Error Status register (0x08)
//
#define TAS5760M_FAULT_CLKE         (0x08)      // Clock error (non-latching)
#define TAS5760M_FAULT_OCE          (0x04)      // Over-current error (latching)
#define TAS5760M_FAULT_DCE          (0x02)      // DC offset error (latching)
#define TAS5760M_FAULT_OTE          (0x01)      // Over-temperature error (latching)
#define TAS5760M_FAULT_NONE         (0x00)      // No fault; normal operation


void tas5760m_interface_init();
uint8_t tas5760m_init();
uint8_t tas5760m_sleep(const uint8_t sleep);
uint8_t tas5760m_shut_down(const uint8_t shut_down);
uint8_t tas5760m_mute(const uint8_t mute);
void tas5760m_isr_fault();
void tas5760m_worker();


#ifdef DEBUG
void tas5760m_dump_registers();
void tas5760m_dump_fault(const uint8_t fault);
#else
#define tas5760m_dump_registers()
#define tas5760m_dump_fault(x)
#endif


#endif
