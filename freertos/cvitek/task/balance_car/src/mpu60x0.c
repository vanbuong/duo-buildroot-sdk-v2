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
#define REG_ACCEL_CONFIG2	0x1D	/* MPU6500 family only */
#define REG_ACCEL_XOUT_H	0x3B
#define REG_SIGNAL_PATH_RESET	0x68
#define REG_PWR_MGMT_1		0x6B
#define REG_WHO_AM_I		0x75

/* Register values written by init (and verified by readback). */
#define CFG_DLPF		0x03	/* gyro ~41-42 Hz, 1 kHz internal rate  */
#define CFG_GYRO_500DPS		0x08
#define CFG_ACCEL_2G		0x00
#define CFG_SMPLRT_1KHZ		0x00
#define CFG_ACCEL2_DLPF		0x03	/* accel ~44.8 Hz (MPU6500 family)      */
#define CFG_PWR_PLL		0x01	/* wake, PLL with X-gyro reference      */

mpu_variant_t mpu60x0_variant_from_id(uint8_t who)
{
	switch (who) {
	case 0x68:
		return MPU_VARIANT_6050;
	case 0x70: case 0x71: case 0x73:	/* MPU6500, MPU9250, MPU9255 */
	case 0x12: case 0xAF: case 0x98:	/* ICM-20602 / 20608 / 20689 */
		return MPU_VARIANT_6500;
	default:
		return MPU_VARIANT_UNKNOWN;
	}
}

const char *mpu60x0_variant_name(mpu_variant_t v)
{
	return v == MPU_VARIANT_6050 ? "MPU6050" :
	       v == MPU_VARIANT_6500 ? "MPU6500-family" : "unknown";
}

static int mpu_write8(mpu60x0_t *imu, uint8_t reg, uint8_t val)
{
	return poll_i2c_write(imu->i2c_id, imu->addr, reg, &val, 1);
}

static int mpu_read(mpu60x0_t *imu, uint8_t reg, uint8_t *buf, uint16_t len)
{
	return poll_i2c_read(imu->i2c_id, imu->addr, reg, buf, len);
}

/* write + read back; 0 ok, -1 bus error, -2 mismatch */
static int mpu_write_verify(mpu60x0_t *imu, uint8_t reg, uint8_t val, uint8_t mask)
{
	uint8_t got = 0;

	if (mpu_write8(imu, reg, val) || mpu_read(imu, reg, &got, 1))
		return -1;
	if ((got & mask) != (val & mask)) {
		printf("[balance] MPU reg 0x%02x: wrote 0x%02x read 0x%02x\n", reg, val, got);
		return -2;
	}
	return 0;
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

	/* identify first: WHO_AM_I is readable while the part is still asleep */
	if (mpu_read(imu, REG_WHO_AM_I, &who, 1)) {
		printf("[balance] MPU WHO_AM_I read failed\n");
		return -1;
	}
	imu->who_am_i = who;
	imu->variant = mpu60x0_variant_from_id(who);
	if (imu->variant == MPU_VARIANT_UNKNOWN) {
		printf("[balance] unexpected WHO_AM_I=0x%02x, assuming MPU6050 register map\n", who);
		imu->variant = MPU_VARIANT_6050;
	} else {
		printf("[balance] MPU WHO_AM_I=0x%02x -> %s\n", who, mpu60x0_variant_name(imu->variant));
	}

	/* device reset, then reset the analog/digital signal paths (both parts) */
	if (mpu_write8(imu, REG_PWR_MGMT_1, 0x80)) {
		printf("[balance] MPU reset write failed\n");
		return -1;
	}
	mdelay(100);
	mpu_write8(imu, REG_SIGNAL_PATH_RESET, 0x07);
	mdelay(100);

	ret = mpu_write8(imu, REG_PWR_MGMT_1, CFG_PWR_PLL);
	if (ret) {
		printf("[balance] MPU PWR_MGMT_1 write failed (%d)\n", ret);
		return -1;
	}
	mdelay(50);

	/* DLPF ~42 Hz (1 kHz internal rate), +-500 dps, +-2 g, read asynchronously at 200 Hz */
	ret = mpu_write_verify(imu, REG_CONFIG, CFG_DLPF, 0x07);
	if (!ret) ret = mpu_write_verify(imu, REG_GYRO_CONFIG, CFG_GYRO_500DPS, 0x18);
	if (!ret) ret = mpu_write_verify(imu, REG_ACCEL_CONFIG, CFG_ACCEL_2G, 0x18);
	if (!ret) ret = mpu_write_verify(imu, REG_SMPLRT_DIV, CFG_SMPLRT_1KHZ, 0xFF);
	if (!ret) ret = mpu_write_verify(imu, REG_PWR_MGMT_1, CFG_PWR_PLL, 0x07);
	if (ret)
		return ret;

	if (imu->variant == MPU_VARIANT_6500) {
		/* accel DLPF is a separate register here; a clone that reports a 6500 id but
		 * ignores it only gets a wider accel bandwidth, so this is a warning */
		if (mpu_write_verify(imu, REG_ACCEL_CONFIG2, CFG_ACCEL2_DLPF, 0x0F))
			printf("[balance] warning: ACCEL_CONFIG2 not accepted (accel bandwidth wider than planned)\n");
	}
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
	/* datasheet conversions differ: MPU6050 T = raw/340 + 36.53, MPU6500 T = raw/333.87 + 21 */
	if (imu->variant == MPU_VARIANT_6500)
		out->temp_c = imu->temp / 333.87f + 21.0f;
	else
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
