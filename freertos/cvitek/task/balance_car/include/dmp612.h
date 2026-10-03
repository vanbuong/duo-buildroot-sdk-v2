/*
 * EXPERIMENTAL InvenSense DMP (MotionApps 6.12) support, selected with est_mode 2.
 *
 * The firmware image is NOT part of this repository: Linux reads it from a file
 * (/mnt/data/bc_dmp612.bin, packed by `bcctl dmp-pack` from the ElectronicCats
 * MPU6050 library) and hands it over through the shared window. The register
 * sequence follows MPU6050_6Axis_MotionApps612.cpp (dmpInitialize). Written for
 * the MPU6050; use on the MPU6500 family is untested. Nothing here has run on
 * real hardware. The estimator only uses the DMP angle while it agrees with the
 * always-running complementary filter (docs/balance_car/03 section 3.5).
 */
#ifndef BC_DMP612_H
#define BC_DMP612_H

#include <stdint.h>
#include "mpu60x0.h"

#define DMP612_CODE_SIZE	3062
#define DMP612_PACKET_SIZE	28
#define DMP612_MAX_PACKETS	8	/* per poll; more means we fell behind */
#define DMP612_FIFO_SIZE	1024
#define DMP612_GYRO_LSB		16.4f	/* the DMP needs +-2000 deg/s */

typedef struct {
	float qw, qx, qy, qz;			/* unit quaternion                  */
	int16_t gx, gy, gz, ax, ay, az;		/* raw (+-2000 deg/s, +-2 g)        */
} dmp_sample_t;

typedef struct {
	uint32_t packets;
	uint32_t bad_packets;			/* quaternion not unit length       */
	uint32_t overflows;			/* FIFO reset because we lagged     */
} dmp_stats_t;

/* 0 ok, -1 bus error, -2 image write verify failed, -3 wrong image size */
int dmp612_load(mpu60x0_t *imu, const uint8_t *code, uint32_t len);

/*
 * Drain the FIFO and return the newest valid packet.
 * 0 new sample, 1 nothing new, -1 bus error, -2 FIFO overflow/lag (FIFO reset).
 */
int dmp612_poll(mpu60x0_t *imu, dmp_sample_t *out, dmp_stats_t *st);

/* Parse one 28-byte packet. 0 ok, -1 quaternion norm outside 1 +- 0.1. */
int dmp612_parse(const uint8_t pkt[DMP612_PACKET_SIZE], dmp_sample_t *out);

/* Gravity direction in the sensor frame, in g (same sign as a resting accelerometer). */
void dmp612_gravity(const dmp_sample_t *s, float g[3]);

#endif
