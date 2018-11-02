/*
    pca9632.h - definitions relating to the driver for the PCA9632 I2C RGBA LED driver.

    Stuart Wallace <stuartw@atom.net>, October 2018.
*/

#include "dev/pca9632.h"
#include "lib/debug.h"
#include "lib/twi.h"


#ifdef WITH_PCA9632

#define PCA9632_I2C_ADDR            (0x62)      // Default address for PCA9632
#define PCA9632_ALL_CALL_I2C_ADDR   (0x70)      // Broadcast address for PCA9632 ICs
#define PCA9632_SWRESET_I2C_ADDR    (0x03)      // Address used for software reset

#define PCA9632_SWRESET_BYTE1       (0xa5)      // First magic number used in software reset
#define PCA9632_SWRESET_BYTE2       (0x5a)      // Second magic number used in software reset

// PCA9632Register_t - enumeration of registers in the PCA9632 LED driver IC.
//
typedef enum PCA9632Register
{
    PCA9632RegMode1         = 0x00,     // [RW] Mode register 1
    PCA9632RegMode2         = 0x01,     // [RW] Mode register 2
    PCA9632RegPWM0          = 0x02,     // [RW] Brightness control - LED0
    PCA9632RegPWM1          = 0x03,     // [RW] Brightness control - LED1
    PCA9632RegPWM2          = 0x04,     // [RW] Brightness control - LED2
    PCA9632RegPWM3          = 0x05,     // [RW] Brightness control - LED3
    PCA9632RegGrpPWM        = 0x06,     // [RW] Group duty cycle control
    PCA9632RegGrpFreq       = 0x07,     // [RW] Group frequency
    PCA9632RegLEDOut        = 0x08,     // [RW] LED output state
    PCA9632RegSubAddr1      = 0x09,     // [RW] I2C bus sub-address 1
    PCA9632RegSubAddr2      = 0x0a,     // [RW] I2C bus sub-address 2
    PCA9632RegSubAddr3      = 0x0b,     // [RW] I2C bus sub-address 3
    PCA9632RegAllCallAddr   = 0x0c      // [RW] LED all-call I2C bus address
} PCA9632Register_t;

//
// Mode register 1 (0x00) bits
//
#define PCA9632_MODE1_AI2       (1 << 7)    // [R ] Register auto-increment: 0=disabled, 1=enabled
#define PCA9632_MODE1_AI1       (1 << 6)    // [R ] Auto-increment mode bit 1
#define PCA9632_MODE1_AI0       (1 << 5)    // [R ] Auto-increment mode bit 0
#define PCA9632_MODE1_SLEEP     (1 << 4)    // [RW] Sleep mode: 0=normal, 1=sleep
#define PCA9632_MODE1_SUB1      (1 << 3)    // [RW] Sub-address 1: 0=ignore, 1=respond
#define PCA9632_MODE1_SUB2      (1 << 2)    // [RW] Sub-address 2: 0=ignore, 1=respond
#define PCA9632_MODE1_SUB3      (1 << 1)    // [RW] Sub-address 3: 0=ignore, 1=respond
#define PCA9632_MODE1_ALLCALL   (1 << 0)    // [RW] All-call address: 0=ignore, 1=respond

//
// Mode register 2 (0x01) bits
//
#define PCA9632_MODE2_DMBLNK    (1 << 5)    // [RW] Group control: 0=dimming, 1=blinking
#define PCA9632_MODE2_INVRT     (1 << 4)    // [RW] Output logic state: 0=true, 1=inverted
#define PCA9632_MODE2_OCH       (1 << 3)    // [RW] Outputs change on... 0=STOP, 1=ACK
#define PCA9632_MODE2_OUTDRV    (1 << 2)    // [RW] Output drive: 0=open-drain, 1=totem-pole
#define PCA9632_MODE2_OUTNE1    (1 << 1)    // [RW] unused
#define PCA9632_MODE2_OUTNE0    (1 << 0)    // [RW] unused

//
// LED driver output state (0x08)
//
#define PCA9632_LEDOUT_LDR3_SHIFT   (6)     // Offset of mode-control bits for LED driver 3
#define PCA9632_LEDOUT_LDR2_SHIFT   (4)     // Offset of mode-control bits for LED driver 2
#define PCA9632_LEDOUT_LDR1_SHIFT   (2)     // Offset of mode-control bits for LED driver 1
#define PCA9632_LEDOUT_LDR0_SHIFT   (0)     // Offset of mode-control bits for LED driver 0

// PCA9632LEDDriveMode_t - enumeration of the available drive modes for each LED driver output.
typedef enum PCA9632LEDDriveMode
{
    PCA9632LEDDriveModeOff          = 0,    // LED driver is off
    PCA9632LEDDriveModeOn           = 1,    // LED driver is on at maximum brightness
    PCA9632LEDDriveModePWM          = 2,    // PWM enabled for LED driver
    PCA9632LEDDriveModePWMDimBlink  = 3     // PWM and group dim/blink enabled for LED driver
} PCA9632LEDDriveMode_t;


static uint8_t sync_register_read(const PCA9632Register_t reg, uint8_t * const val);
static uint8_t sync_register_write(const PCA9632Register_t reg, const uint8_t val);


// pca9632_init() - initialise the interface with the PCA9632 chip, and initialise the chip itself.
//
uint8_t pca9632_init()
{
    debug_putstr_p("PCA9632: init\n");

    if(!pca9632_software_reset())
        return 0;

    if(!pca9632_sleep(0))               // Attempt to wake the device to configure it
        return 0;

    // Assume the device is directly driving common-cathode LEDs - invert outputs and enable totem-
    // pole drivers.
    if(!sync_register_write(PCA9632RegMode2, PCA9632_MODE2_INVRT | PCA9632_MODE2_OUTDRV))
        return 0;

    // Reset all PCM values
    for(PCA9632Register_t r = PCA9632RegPWM0; r <= PCA9632RegPWM3; ++r)
        if(!sync_register_write(r, 0))
            return 0;

    // Enable PWM and dimming/blinking on all LED drivers
    if(!sync_register_write(PCA9632RegLEDOut,
                            (PCA9632LEDDriveModePWMDimBlink << PCA9632_LEDOUT_LDR3_SHIFT) |
                            (PCA9632LEDDriveModePWMDimBlink << PCA9632_LEDOUT_LDR2_SHIFT) |
                            (PCA9632LEDDriveModePWMDimBlink << PCA9632_LEDOUT_LDR1_SHIFT) |
                            (PCA9632LEDDriveModePWMDimBlink << PCA9632_LEDOUT_LDR0_SHIFT)))
        return 0;

    debug_putstr_p("PCA9632: init done\n");
    return pca9632_sleep(1);        // Put the device to sleep
}


// pca9632_software_reset() - issue a software reset to the PCA9632 device.  This is done by
// performing an I2C "write" transaction to the designated PCA9632 software-reset address (0x03),
// and following the address with two bytes containing magic numbers.  The function returns non-
// zero on success; zero on failure.
//
uint8_t pca9632_software_reset()
{
    return twi_sync_register_write(PCA9632_SWRESET_I2C_ADDR,
                                   PCA9632_SWRESET_BYTE1, PCA9632_SWRESET_BYTE2) == TWICmdSuccess;
}


// pca9632_sleep() - activate (if <sleep> is nonzero) or deactivate (if <sleep> equals zero) sleep
// mode in the PCA9632 IC.
//
uint8_t pca9632_sleep(const uint8_t sleep)
{
    uint8_t mode1_val = 0;

    if(!sync_register_read(PCA9632RegMode1, &mode1_val))
        return 0;

    if(sleep)
        mode1_val |= PCA9632_MODE1_SLEEP;
    else
        mode1_val &= ~PCA9632_MODE1_SLEEP;

    return sync_register_write(PCA9632RegMode1, mode1_val);
}


// pca9632_pwm_set() - set the PWM value of the channel specified by <channel> to the value
// specified by <pwm_val>.  The value normally controls the brightness of the LED connected to the
// channel.  A value of 0 switches the channel fully off, and 0xff switches it fully on.  Returns
// zero on failure, or non-zero on success.
//
uint8_t pca9632_pwm_set(const PCA9632Channel_t channel, const uint8_t pwm_val)
{
    return sync_register_write(PCA9632RegPWM0 + channel, pwm_val);
}


// pca9632_pwm_set_all() - set the PWM values of all four channels to the values supplied.  A
// value of 0 switches the channel fully off, and 0xff switches it fully on.  Returns zero on
// failure, or non-zero on success.
//
uint8_t pca9632_pwm_set_all(const uint8_t val0, const uint8_t val1, const uint8_t val2,
                            const uint8_t val3)
{
    return pca9632_pwm_set(PCA9632Channel0, val0) &&
           pca9632_pwm_set(PCA9632Channel1, val1) &&
           pca9632_pwm_set(PCA9632Channel2, val2) &&
           pca9632_pwm_set(PCA9632Channel3, val3);
}


// pca9632_mode_set() - set the mode of the PCA9632 to "dim" or "blink", according to the value
// supplied in <mode>.  Returns zero on failure, non-zero on success.
//
uint8_t pca9632_mode_set(const PCA9632Mode_t mode)
{
    uint8_t mode2_val = 0;

    if(!sync_register_read(PCA9632RegMode2, &mode2_val))
        return 0;

    if(mode)
        mode2_val |= PCA9632_MODE2_DMBLNK;
    else
        mode2_val &= ~PCA9632_MODE2_DMBLNK;

    return sync_register_write(PCA9632RegMode2, mode2_val);
}


// pca9632_group_pwm_set() - if the device is in "dim" mode, set the brightness of all LED driver
// channels to the value specified by <pwm_val>, where 0=off and 0xff=no dimming.  If the device is
// in "blink" mode, the value in <pwm_val> specifies the ratio of on/off time for all output
// channels: 0=off, 0xff=[98.4% on, 1.6% off].  Returns zero on failure, non-zero on success.
//
uint8_t pca9632_group_pwm_set(const uint8_t pwm_val)
{
    return sync_register_write(PCA9632RegGrpPWM, pwm_val);
}


// pca9632_group_freq_set() - if the device is in "blink" mode, set the blinking frequency
// according to the value in <freq>.  The blink period equals ((<freq> + 1) / 24) seconds.  If the
// device is in "dim" mode, this function has no effect.  Returns zero on failure, non-zero on
// success.
//
uint8_t pca9632_group_freq_set(const uint8_t freq)
{
    return sync_register_write(PCA9632RegGrpFreq, freq);
}


// sync_register_read() - convenience wrapper around twi_sync_register_read().  Avoids the need to
// specify PCA9632_I2C_ADDR in every call.
//
static uint8_t sync_register_read(const PCA9632Register_t reg, uint8_t * const val)
{
    return twi_sync_register_read(PCA9632_I2C_ADDR, reg, val) == TWICmdSuccess;
}


// sync_register_write() - convenience wrapper around twi_sync_register_write().  Avoids the need
// to specify PCA9632_I2C_ADDR in every call.
//
static uint8_t sync_register_write(const PCA9632Register_t reg, const uint8_t val)
{
    return twi_sync_register_write(PCA9632_I2C_ADDR, reg, val) == TWICmdSuccess;
}


// Debug functions from here on
//
#ifdef DEBUG

// pca9632_dump_registers() - if a debug build is running, dump the contents of the PCA9632
// internal registers to the debug console.
//
void pca9632_dump_registers()
{
    debug_putstr_p("\n== PCA9632 register dump ==\n");
    twi_dump_registers(PCA9632_I2C_ADDR, PCA9632RegAllCallAddr);
}

#endif // DEBUG
#endif // WITH_PCA9632
