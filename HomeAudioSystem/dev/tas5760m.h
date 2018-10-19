#ifndef DEV_TAS5760M_H_INC
#define DEV_TAS5760M_H_INC
/*
    tas5760m.h - declarations relating to the driver for the TAS5760M I2S-input class-D audio
    amplifier IC.  This driver currently assumes that the amplifier is configured in PBTL mode.

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "platform.h"

#ifdef WITH_TAS5760M

#include <stdint.h>


// TAS5760M error constants, from the Fault Configuration / Error Status register (0x08)
//
#define TAS5760M_FAULT_CLKE         (0x08)      // Clock error (non-latching)
#define TAS5760M_FAULT_OCE          (0x04)      // Over-current error (latching)
#define TAS5760M_FAULT_DCE          (0x02)      // DC offset error (latching)
#define TAS5760M_FAULT_OTE          (0x01)      // Over-temperature error (latching)
#define TAS5760M_FAULT_NONE         (0x00)      // No fault; normal operation

// TAS5760MPBTLChannel_t - enumeration of arguments to the tas5760m_set_pbtl_channel() function,
// which specifies the channel to be amplified when the amplifier is configured in parallel bridge-
// tied-load (PBTL) mode.
//
typedef enum TAS5760MPBTLChannel
{
    TAS5760MPBTLChannelRight    = 0,
    TAS5760MPBTLChannelLeft     = 1
} TAS5760MPBTLChannel_t;


void tas5760m_interface_init();
uint8_t tas5760m_init();
uint8_t tas5760m_sleep(const uint8_t sleep);
uint8_t tas5760m_shut_down(const uint8_t shut_down);
uint8_t tas5760m_mute(const uint8_t mute);
uint8_t tas5760m_set_gain(const int8_t gain);
uint8_t tas5760m_set_pbtl_channel(const TAS5760MPBTLChannel_t channel);
void tas5760m_isr_fault();
void tas5760m_worker();


#ifdef DEBUG
void tas5760m_dump_registers();
void tas5760m_dump_fault(const uint8_t fault);
#else
#define tas5760m_dump_registers()
#define tas5760m_dump_fault(x)
#endif

#endif // WITH_TAS5760M
#endif
