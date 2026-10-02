#include "mpu60x0.h"
#include "poll_i2c.h"
#include "delay.h"
#include "printf.h"
#include <math.h>
#include <string.h>

#define REG_SMPLRT_DIV		0x19
#define REG_CONFIG		0x1A
#define REG_GYRO_CONFIG		0x1B
#define REG_ACCEL_CONFIG	0x1C
#define REG_ACCEL_CONFIG2	0x1D	/* MPU6500/9250 only */
#define REG_ACCEL_XOUT_H	0x3B
#define REG_PWR_MGMT_1		0x6B
#define REG_WHO_AM_I		0x75

#define WHO_AM_I_MPU6050	0x68
#define WHO_AM_I_MPU6500	0x70
#define WHO_AM_I_MPU9250	0x71

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

	memset(imu, 0, sizeof(*imu));
	imu->i2c_id = i2c_id;
	imu->addr = addr;

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
	imu->who_am_i = who;
	if (who != WHO_AM_I_MPU6050 && who != WHO_AM_I_MPU6500 &&
	    who != WHO_AM_I_MPU9250 && who != 0x98)
		printf("[balance] unexpected WHO_AM_I=0x%02x (continuing)\n", who);
	else
		printf("[balance] MPU WHO_AM_I=0x%02x OK\n", who);

	/* DLPF 42 Hz (1 kHz internal rate); read asynchronously at 200 Hz */
	mpu_write8(imu, REG_CONFIG, 0x03);
	mpu_write8(imu, REG_GYRO_CONFIG, 0x08);		/* +-500 dps  */
	mpu_write8(imu, REG_ACCEL_CONFIG, 0x00);	/* +-2 g      */
	mpu_write8(imu, REG_SMPLRT_DIV, 0x00);		/* 1 kHz      */
	if (who == WHO_AM_I_MPU6500 || who == WHO_AM_I_MPU9250 || who == 0x98)
		mpu_write8(imu, REG_ACCEL_CONFIG2, 0x03); /* accel DLPF ~44 Hz */

	return 0;
}

int mpu60x0_read(mpu60x0_t *imu)
{
	uint8_t buf[14];
	int ret;

	ret = mpu_read(imu, REG_ACCEL_XOUT_H, buf, 14);
	if (ret)
		return ret;

	memcpy(imu->raw, buf, sizeof(buf));
	imu->ax = (int16_t)((buf[0] << 8) | buf[1]);
	imu->ay = (int16_t)((buf[2] << 8) | buf[3]);
	imu->az = (int16_t)((buf[4] << 8) | buf[5]);
	imu->temp = (int16_t)((buf[6] << 8) | buf[7]);
	imu->gx = (int16_t)((buf[8] << 8) | buf[9]);
	imu->gy = (int16_t)((buf[10] << 8) | buf[11]);
	imu->gz = (int16_t)((buf[12] << 8) | buf[13]);
	return 0;
}

void mpu60x0_scale(const mpu60x0_t *imu, mpu60x0_scaled_t *out)
{
	out->ax = imu->ax / MPU_ACCEL_LSB_PER_G;
	out->ay = imu->ay / MPU_ACCEL_LSB_PER_G;
	out->az = imu->az / MPU_ACCEL_LSB_PER_G;
	out->gx = imu->gx / MPU_GYRO_LSB_PER_DPS - imu->gyro_bias[0];
	out->gy = imu->gy / MPU_GYRO_LSB_PER_DPS - imu->gyro_bias[1];
	out->gz = imu->gz / MPU_GYRO_LSB_PER_DPS - imu->gyro_bias[2];
	out->temp_c = imu->temp / 340.0f + 36.53f;
}

int mpu60x0_calibrate(mpu60x0_t *imu, int samples)
{
	double sg[3] = { 0, 0, 0 }, sg2[3] = { 0, 0, 0 }, sa[3] = { 0, 0, 0 };
	float rms, mean, an, bias[3], amean[3];
	int i, k;

	if (samples <= 1)
		samples = 200;
	printf("[balance] calibrating IMU (%d samples) - keep still\n", samples);
	for (i = 0; i < samples; i++) {
		double g[3], a[3];

		if (mpu60x0_read(imu))
			return -1;
		g[0] = imu->gx / MPU_GYRO_LSB_PER_DPS;
		g[1] = imu->gy / MPU_GYRO_LSB_PER_DPS;
		g[2] = imu->gz / MPU_GYRO_LSB_PER_DPS;
		a[0] = imu->ax / MPU_ACCEL_LSB_PER_G;
		a[1] = imu->ay / MPU_ACCEL_LSB_PER_G;
		a[2] = imu->az / MPU_ACCEL_LSB_PER_G;
		for (k = 0; k < 3; k++) {
			sg[k] += g[k];
			sg2[k] += g[k] * g[k];
			sa[k] += a[k];
		}
		mdelay(5);
	}

	for (k = 0; k < 3; k++) {
		mean = (float)(sg[k] / samples);
		rms = (float)sqrt(fmax(0.0, sg2[k] / samples - (double)mean * mean));
		if (rms > 1.0f) {
			printf("[balance] calibration rejected: gyro[%d] rms %.2f\n", k, rms);
			return -2;
		}
		bias[k] = mean;
		amean[k] = (float)(sa[k] / samples);
	}
	an = sqrtf(amean[0] * amean[0] + amean[1] * amean[1] + amean[2] * amean[2]);
	if (an < 0.9f || an > 1.1f) {
		printf("[balance] calibration rejected: |a|=%.2f g\n", an);
		return -2;
	}
	memcpy(imu->gyro_bias, bias, sizeof(bias));
	memcpy(imu->accel_mean, amean, sizeof(amean));
	imu->calibrated = 1;
	printf("[balance] gyro bias %.3f %.3f %.3f dps\n", imu->gyro_bias[0],
	       imu->gyro_bias[1], imu->gyro_bias[2]);
	return 0;
}
