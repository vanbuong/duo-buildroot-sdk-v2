#ifndef BALANCE_POLL_I2C_H
#define BALANCE_POLL_I2C_H

#include <stdint.h>

int poll_i2c_init(uint8_t bus_id);
int poll_i2c_write(uint8_t bus_id, uint8_t addr, uint8_t reg,
		   const uint8_t *data, uint16_t len);
int poll_i2c_read(uint8_t bus_id, uint8_t addr, uint8_t reg,
		  uint8_t *data, uint16_t len);

#endif /* BALANCE_POLL_I2C_H */
