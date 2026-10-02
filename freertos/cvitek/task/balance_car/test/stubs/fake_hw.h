/*
 * Fake hardware used by host unit tests.
 *  - GPIO: every pin is a settable level (fake_gpio_set) that the firmware
 *    reads through gpio_get_value().
 *  - I2C: poll_i2c_read() serves bytes from a 256-entry register file, and
 *    records writes; failures can be injected.
 */
#ifndef FAKE_HW_H
#define FAKE_HW_H
#include <stdint.h>

void fake_gpio_reset(void);
void fake_gpio_set(int pin, int level);
int  fake_gpio_get(int pin);

void fake_i2c_reset(void);
void fake_i2c_set_reg(uint8_t reg, uint8_t val);
void fake_i2c_set_accel_gyro(int16_t ax, int16_t ay, int16_t az,
			     int16_t gx, int16_t gy, int16_t gz);
uint8_t fake_i2c_get_reg(uint8_t reg);
void fake_i2c_fail_reads(int fail);
int  fake_i2c_write_count(void);
#endif
