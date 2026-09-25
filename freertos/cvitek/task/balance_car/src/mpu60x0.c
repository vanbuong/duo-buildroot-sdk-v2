#include "mpu60x0.h"
#include "poll_i2c.h"
#include "delay.h"
#include "printf.h"
#include <math.h>

#define REG_SMPLRT_DIV		0x19
#define REG_CONFIG		0x1A
#define REG_GYRO_CONFIG		0x1B
#define REG_ACCEL_CONFIG	0x1C
#define REG_ACCEL_XOUT_H	0x3B
#define REG_PWR_MGMT_1		0x6B
#define REG_WHO_AM_I		0x75

#define WHO_AM_I_MPU6050	0x68
#define WHO_AM_I_MPU6500	0x70
#define WHO_AM_I_MPU9250	0x71

#define ACCEL_LSB_2G		16384.0f
#define GYRO_LSB_250DPS		131.0f
#define RAD2DEG			57.2957795f

static int mpu_write8(mpu60x0_t *imu, uint8_t reg, uint8_t val)
{
	return poll_i2c_write(imu->i2c_id, imu->addr, reg, &val, 1);
}

static int mpu_read(mpu60x0_t *imu, uint8_t reg, uint8_t *buf, uint16_t len)
{
	return poll_i2c_read(imu->i2c_id, imu->addr, reg, buf, len);
}

int mpu60x0_init(mpu60x0_t *imu, uint8_t i2c_id, uint8_t addr)
{
	uint8_t who = 0;
	int ret;

	imu->i2c_id = i2c_id;
	imu->addr = addr;
	imu->angle_deg = 0.0f;
	imu->gyro_bias_y = 0.0f;

	if (poll_i2c_init(i2c_id))
		return -1;
	mdelay(10);

	ret = mpu_write8(imu, REG_PWR_MGMT_1, 0x01);
	if (ret) {
		printf("[balance] MPU PWR_MGMT_1 write failed (%d)\n", ret);
		return -1;
	}
	mdelay(50);

	ret = mpu_read(imu, REG_WHO_AM_I, &who, 1);
	if (ret) {
		printf("[balance] MPU WHO_AM_I read failed (%d)\n", ret);
		return -1;
	}
	if (who != WHO_AM_I_MPU6050 && who != WHO_AM_I_MPU6500 &&
	    who != WHO_AM_I_MPU9250 && who != 0x98) {
		printf("[balance] unexpected WHO_AM_I=0x%02x (continuing)\n", who);
	} else {
		printf("[balance] MPU WHO_AM_I=0x%02x OK\n", who);
	}

	mpu_write8(imu, REG_CONFIG, 0x03);
	mpu_write8(imu, REG_GYRO_CONFIG, 0x00);
	mpu_write8(imu, REG_ACCEL_CONFIG, 0x00);
	mpu_write8(imu, REG_SMPLRT_DIV, 0x04);

	return 0;
}

int mpu60x0_read(mpu60x0_t *imu)
{
	uint8_t buf[14];
	int ret;

	ret = mpu_read(imu, REG_ACCEL_XOUT_H, buf, 14);
	if (ret)
		return ret;

	imu->ax = (int16_t)((buf[0] << 8) | buf[1]);
	imu->ay = (int16_t)((buf[2] << 8) | buf[3]);
	imu->az = (int16_t)((buf[4] << 8) | buf[5]);
	imu->gx = (int16_t)((buf[8] << 8) | buf[9]);
	imu->gy = (int16_t)((buf[10] << 8) | buf[11]);
	imu->gz = (int16_t)((buf[12] << 8) | buf[13]);
	return 0;
}

void mpu60x0_update_angle(mpu60x0_t *imu, float dt)
{
	float ax = imu->ax / ACCEL_LSB_2G;
	float ay = imu->ay / ACCEL_LSB_2G;
	float az = imu->az / ACCEL_LSB_2G;
	float gyro_y = (imu->gy / GYRO_LSB_250DPS) - imu->gyro_bias_y;
	float accel_pitch;
	const float alpha = 0.98f;

	accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
	imu->angle_deg = alpha * (imu->angle_deg + gyro_y * dt) +
			 (1.0f - alpha) * accel_pitch;
}

int mpu60x0_calibrate_gyro(mpu60x0_t *imu, int samples)
{
	int i;
	float sum = 0.0f;

	if (samples <= 0)
		samples = 200;

	printf("[balance] calibrating gyro (%d samples)...\n", samples);
	for (i = 0; i < samples; i++) {
		if (mpu60x0_read(imu))
			return -1;
		sum += imu->gy / GYRO_LSB_250DPS;
		mdelay(5);
	}
	imu->gyro_bias_y = sum / (float)samples;
	printf("[balance] gyro_bias_y=%.3f dps\n", imu->gyro_bias_y);
	return 0;
}
