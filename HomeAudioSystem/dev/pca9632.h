#ifndef DEV_PCA9632_H_INC
#define DEV_PCA9632_H_INC
/*
    pca9632.h - declarations relating to the driver for the PCA9632 I2C RGBA LED driver.

    Stuart Wallace <stuartw@atom.net>, October 2018.
*/

#include "platform.h"
#include <stdint.h>

#ifdef WITH_PCA9632


// PCA9632Channel_t - specifies a channel (LED driver output).
typedef enum PCA9632Channel
{
    PCA9632Channel0 = 0,
    PCA9632Channel1,
    PCA9632Channel2,
    PCA9632Channel3
} PCA9632Channel_t;


// PCA9632Mode_t - specifies the mode (dimming or blinking) of operation of the PCA9632 IC.
typedef enum PCA9632Mode
{
    PCA9632ModeDim = 0,
    PCA9632ModeBlink = 1
} PCA9632Mode_t;


uint8_t pca9632_init();
uint8_t pca9632_software_reset();
uint8_t pca9632_sleep(const uint8_t sleep);
uint8_t pca9632_pwm_set(const PCA9632Channel_t channel, const uint8_t pwm_val);
uint8_t pca9632_pwm_set_all(const uint8_t val0, const uint8_t val1, const uint8_t val2,
                            const uint8_t val3);
uint8_t pca9632_mode_set(const PCA9632Mode_t mode);
uint8_t pca9632_group_pwm_set(const uint8_t pwm_val);
uint8_t pca9632_group_freq_set(const uint8_t freq);

#ifdef DEBUG
void pca9632_dump_registers();
#else
#define pca9632_dump_registers()
#endif

#endif // WITH_PCA9632
#endif // DEV_PCA9632_H_INC
