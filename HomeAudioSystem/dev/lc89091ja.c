/*
    lc89091ja.c  - definitions related to the driver for the LC89091JA-H digital audio interface
    receiver.

    Stuart Wallace <stuartw@atom.net>, September 2018.

    Ideally this driver would use the LC89091JA-H's I2C interface to configure the device.  I have,
    however, been unable to make this interface work reliably.  This is probably a consequence of a
    bug in the lib/twi.c module, although this module drives other I2C devices (e.g. the TAS5760M)
    perfectly reliably.  Fortunately the LC89091JA-H can, for the purposes of this project, be
    driven in "hardware mode" without recourse to the internal registers.  Consequently this driver
    uses GPIO pins to monitor the status of the chip.
*/

#include "lc89091ja.h"
#include "lib/debug.h"
#include "lib/gpio.h"
#include "platform.h"


#ifdef WITH_LC89091JA

#define LC89091JA_I2C_ADDR      (0x12)      // I2C address of the LC89091JA receiver


// lc89091ja_init() - initialise the interface with the LC89091JA-H digital audio receiver.  This
// is a trivial interface: the uC monitors the ERR output pin on the LC89091JA-H.
//
void lc89091ja_init()
{
    debug_putstr_p("LC89091JA: init\n");

    gpio_make_input(PIN_RX_ERR);
}


// lc89091_get_err_state() - return the state of the LC89091JA-H ERR pin, which is asserted when
// a timing error occurs.  Returns non-zero if the pin is asserted, zero otherwise.
//
uint8_t lc89091_get_error_state()
{
    return gpio_read(PIN_RX_ERR);
}

#endif // WITH_LC89091JA
