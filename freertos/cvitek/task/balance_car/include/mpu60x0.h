#ifndef BALANCE_MPU60X0_H
#define BALANCE_MPU60X0_H

#include <stdint.h>

typedef struct {
	uint8_t i2c_id;
	uint8_t addr;
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	float angle_deg;	/* complementary pitch estimate */
	float gyro_bias_y;
} mpu60x0_t;

int mpu60x0_init(mpu60x0_t *imu, uint8_t i2c_id, uint8_t addr);
int mpu60x0_read(mpu60x0_t *imu);
void mpu60x0_update_angle(mpu60x0_t *imu, float dt);
int mpu60x0_calibrate_gyro(mpu60x0_t *imu, int samples);

#endif /* BALANCE_MPU60X0_H */
