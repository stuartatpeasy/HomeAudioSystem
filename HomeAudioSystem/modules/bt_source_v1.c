/*
    bt_source_v1.c - definitions relating to the firmware for V1 of the Bluetooth source module,
    which is based on a RN52-I/RM116 module.

    Stuart Wallace <stuartw@atom.net>, November 2018.
*/

#include "bt_source_v1.h"
#include "core/control.h"
#include "dev/pca9632.h"
#include "lib/twi.h"
#include "util/irq.h"


#ifdef MODULE_BT_SOURCE_V1


// firmware_main() - firmware entry point
//
void firmware_main()
{
    twi_configure_master(PinsetDefault);
    twi_master_enable(1);
    twi_set_clock(TWISpeed_400kHz);

    interrupt_enable_increment();

    pca9632_init();                         // Initialise the PCA9632 LED driver
    ctrl_init();                            // Initialise the control interface

    // Set LED to red during initialisation
    pca9632_sleep(0);
    pca9632_pwm_set_all(0xff, 0x00, 0x00, 0x00);

    // Worker loop
    while(1)
    {

    }
}


#endif // MODULE_BT_SOURCE_V1
