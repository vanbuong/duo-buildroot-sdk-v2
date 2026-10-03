#ifndef BALANCE_MPU60X0_H
#define BALANCE_MPU60X0_H

#include <stdint.h>

/* Full-scale ranges configured by mpu60x0_init() (docs/balance_car/03 2.2). */
#define MPU_ACCEL_LSB_PER_G	16384.0f	/* +-2 g    */
#define MPU_GYRO_LSB_PER_DPS	65.5f		/* +-500 dps */

/*
 * Supported parts. Both have a hardware digital low-pass filter (DLPF) that the
 * driver configures to ~42-44 Hz; the on-chip DMP is NOT used (it needs the
 * proprietary InvenSense firmware blob, and the estimator needs raw data for
 * its accelerometer gate). docs/balance_car/03 section 2.
 */
typedef enum {
	MPU_VARIANT_UNKNOWN = 0,
	MPU_VARIANT_6050,	/* WHO_AM_I 0x68                                        */
	MPU_VARIANT_6500,	/* 0x70 MPU6500, 0x71 MPU9250, 0x73 MPU9255, and the
				 * register-compatible ICM-20602 (0x12), ICM-20608 (0xAF),
				 * ICM-20689 (0x98)                                     */
} mpu_variant_t;

mpu_variant_t mpu60x0_variant_from_id(uint8_t who_am_i);
const char *mpu60x0_variant_name(mpu_variant_t v);

typedef struct {
	uint8_t i2c_id;
	uint8_t addr;
	uint8_t who_am_i;
	mpu_variant_t variant;
	int16_t ax, ay, az, temp;
	int16_t gx, gy, gz;
	uint8_t raw[14];		/* last burst, for stuck-data detection */
	/* boot calibration (deg/s, g) */
	float gyro_bias[3];
	float accel_mean[3];
	int calibrated;
	float gyro_lsb;			/* LSB per deg/s of the configured range (0 = default) */
} mpu60x0_t;

typedef struct {
	float ax, ay, az;		/* g                              */
	float gx, gy, gz;		/* deg/s, boot bias removed       */
	float temp_c;
} mpu60x0_scaled_t;

/*
 * Detects the variant, resets the device, applies the variant-specific setup and
 * reads every critical register back. Returns 0, -1 on a bus error, -2 if a
 * register did not take the value written (wrong/faulty part or bus).
 */
int mpu60x0_init(mpu60x0_t *imu, uint8_t i2c_id, uint8_t addr);
int mpu60x0_read(mpu60x0_t *imu);

/* Raw register access for sibling drivers (DMP). 0 on success. */
int mpu60x0_reg_write(mpu60x0_t *imu, uint8_t reg, uint8_t val);
int mpu60x0_reg_read(mpu60x0_t *imu, uint8_t reg, uint8_t *buf, uint16_t len);
int mpu60x0_reg_write_n(mpu60x0_t *imu, uint8_t reg, const uint8_t *buf, uint16_t len);
void mpu60x0_scale(const mpu60x0_t *imu, mpu60x0_scaled_t *out);
/*
 * Average `samples` frames (robot still): gyro bias for all axes and the mean
 * accel vector. Returns 0 on success, -1 bus error, -2 if the robot moved
 * (gyro RMS > 1 deg/s or |accel| not within 1 +- 0.1 g).
 */
int mpu60x0_calibrate(mpu60x0_t *imu, int samples);

#endif /* BALANCE_MPU60X0_H */
