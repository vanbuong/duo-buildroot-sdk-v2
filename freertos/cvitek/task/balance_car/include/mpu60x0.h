#ifndef BALANCE_MPU60X0_H
#define BALANCE_MPU60X0_H

#include <stdint.h>

/* Full-scale ranges configured by mpu60x0_init() (docs/balance_car/03 2.2). */
#define MPU_ACCEL_LSB_PER_G	16384.0f	/* +-2 g    */
#define MPU_GYRO_LSB_PER_DPS	65.5f		/* +-500 dps */

typedef struct {
	uint8_t i2c_id;
	uint8_t addr;
	uint8_t who_am_i;
	int16_t ax, ay, az, temp;
	int16_t gx, gy, gz;
	uint8_t raw[14];		/* last burst, for stuck-data detection */
	/* boot calibration (deg/s, g) */
	float gyro_bias[3];
	float accel_mean[3];
	int calibrated;
} mpu60x0_t;

typedef struct {
	float ax, ay, az;		/* g                              */
	float gx, gy, gz;		/* deg/s, boot bias removed       */
	float temp_c;
} mpu60x0_scaled_t;

int mpu60x0_init(mpu60x0_t *imu, uint8_t i2c_id, uint8_t addr);
int mpu60x0_read(mpu60x0_t *imu);
void mpu60x0_scale(const mpu60x0_t *imu, mpu60x0_scaled_t *out);
/*
 * Average `samples` frames (robot still): gyro bias for all axes and the mean
 * accel vector. Returns 0 on success, -1 bus error, -2 if the robot moved
 * (gyro RMS > 1 deg/s or |accel| not within 1 +- 0.1 g).
 */
int mpu60x0_calibrate(mpu60x0_t *imu, int samples);

#endif /* BALANCE_MPU60X0_H */
