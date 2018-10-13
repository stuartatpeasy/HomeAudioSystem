/*
    rtc.c - definitions for helper functions relating to the uC's real-time counter

    Stuart Wallace <stuartw@atom.net>, September 2018.
*/

#include "rtc.h"

// rtc_pitctrla_sync_wait() - Macro which can be used to wait until the uC has finished
// synchronising the PITCTRLA register.  This must be done before any update to PITCTRLA.
//
#define rtc_pitctrla_sync_wait()                \
    do                                          \
    {                                           \
        while(RTC_PITSTATUS & RTC_CTRLBUSY_bm)  \
            ;                                   \
    } while(0)


// rtc_set_clock() - specify the clock source for the real-time counter (RTC).
//
void rtc_set_clock(const RTCClkSel_t clock)
{
    RTC_CLKSEL = (RTC_CLKSEL & ~RTC_CLKSEL_gm) | clock;
}


// rtc_set_prescaler() - set the division ratio of the real-time counter (RTC) prescaler.  The
// selected RTC clock will be divided by this value to derive the RTC period.
//
void rtc_set_prescaler(const RTCPrescaler_t prescaler)
{
    RTC_CTRLA = (RTC_CTRLA & ~RTC_PRESCALER_gm) | prescaler;
}


// rtc_enable() - enable (if <enable> is non-zero) or disable (if <enable> equals zero) the real-
// time counter (RTC).
//
void rtc_enable(const uint8_t enable)
{
    if(enable)
        RTC_CTRLA |= RTC_RTCEN_bm;
    else
        RTC_CTRLA &= ~RTC_RTCEN_bm;
}


// rtc_pit_enable() enable (if <enable> is non-zero) or disable (if <enable> equals zero) the real-
// time counter (RTC)'s periodic interrupt timer (PIT).
//
void rtc_pit_enable(const uint8_t enable)
{
    rtc_pitctrla_sync_wait();
    if(enable)
        RTC_PITCTRLA |= RTC_PITEN_bm;
    else
        RTC_PITCTRLA &= ~RTC_PITEN_bm;
}


// rtc_pit_set_period() - set the "period" (number of RTC clock cycles) between successive PIT
// interrupts.
//
void rtc_pit_set_period(const RTCPITPeriod_t period)
{
    rtc_pitctrla_sync_wait();
    RTC_PITCTRLA = (RTC_PITCTRLA & ~RTC_PERIOD_gm) | period;
}


// rtc_pit_irq_enable() - enable (if <enable> is non-zero) or disable (if <enable> equals zero) the
// real-time counter (RTC)'s periodic interrupt timer (PIT) interrupt.
//
void rtc_pit_irq_enable(const uint8_t enable)
{
    if(enable)
        RTC_PITINTCTRL |= RTC_PI_bm;
    else
        RTC_PITINTCTRL &= ~RTC_PI_bm;
}


// rtc_pit_irq_acknowledge() - acknowledge a PIT interrupt, enabling the next interrupt to occur
// after the PIT period has elapsed.
//
void rtc_pit_irq_acknowledge()
{
    RTC_PITINTFLAGS = 1;    // Clear periodic interrupt flag
}
