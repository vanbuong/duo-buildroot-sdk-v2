#include <math.h>
#include <string.h>
#include "dmp612.h"
#include "delay.h"
#include "printf.h"

#define RA_SMPLRT_DIV	0x19
#define RA_CONFIG	0x1A
#define RA_GYRO_CONFIG	0x1B
#define RA_ACCEL_CONFIG	0x1C
#define RA_FIFO_EN	0x23
#define RA_INT_PIN_CFG	0x37
#define RA_INT_ENABLE	0x38
#define RA_USER_CTRL	0x6A
#define RA_PWR_MGMT_1	0x6B
#define RA_BANK_SEL	0x6D
#define RA_MEM_START	0x6E
#define RA_MEM_RW	0x6F
#define RA_DMP_CFG_1	0x70
#define RA_FIFO_COUNTH	0x72
#define RA_FIFO_RW	0x74

#define CHUNK		16
#define Q_SCALE		16384.0f

static int wr(mpu60x0_t *imu, uint8_t reg, uint8_t val)
{
	return mpu60x0_reg_write(imu, reg, val) ? -1 : 0;
}

/* write the image in 16-byte chunks that never cross a 256-byte bank, verify each */
static int write_image(mpu60x0_t *imu, const uint8_t *code, uint32_t len)
{
	uint32_t pos = 0;
	uint8_t back[CHUNK];

	while (pos < len) {
		uint8_t bank = (uint8_t)(pos >> 8), addr = (uint8_t)(pos & 0xff);
		uint32_t n = len - pos;

		if (n > CHUNK)
			n = CHUNK;
		if (n > 256u - addr)
			n = 256u - addr;
		if (wr(imu, RA_BANK_SEL, bank) || wr(imu, RA_MEM_START, addr) ||
		    mpu60x0_reg_write_n(imu, RA_MEM_RW, code + pos, (uint16_t)n))
			return -1;
		if (wr(imu, RA_BANK_SEL, bank) || wr(imu, RA_MEM_START, addr) ||
		    mpu60x0_reg_read(imu, RA_MEM_RW, back, (uint16_t)n))
			return -1;
		if (memcmp(back, code + pos, n) != 0)
			return -2;
		pos += n;
	}
	return 0;
}

int dmp612_load(mpu60x0_t *imu, const uint8_t *code, uint32_t len)
{
	uint8_t cfg[2];
	int rc;

	if (len != DMP612_CODE_SIZE || !code)
		return -3;
	/* same order as dmpInitialize() of MotionApps 6.12 */
	if (wr(imu, RA_PWR_MGMT_1, 0x80))		/* device reset */
		return -1;
	mdelay(100);
	if (wr(imu, RA_USER_CTRL, 0x07))		/* signal path reset */
		return -1;
	mdelay(100);
	if (wr(imu, RA_PWR_MGMT_1, 0x01) || wr(imu, RA_INT_ENABLE, 0x00) ||
	    wr(imu, RA_FIFO_EN, 0x00) || wr(imu, RA_ACCEL_CONFIG, 0x00) ||
	    wr(imu, RA_INT_PIN_CFG, 0x80) || wr(imu, RA_PWR_MGMT_1, 0x01) ||
	    wr(imu, RA_SMPLRT_DIV, 0x01) || wr(imu, RA_CONFIG, 0x01))
		return -1;
	rc = write_image(imu, code, len);
	if (rc)
		return rc;
	cfg[0] = 0x04;					/* DMP program start address 0x0400 */
	cfg[1] = 0x00;
	if (mpu60x0_reg_write_n(imu, RA_DMP_CFG_1, cfg, 2))
		return -1;
	if (wr(imu, RA_GYRO_CONFIG, 0x18) ||		/* +-2000 deg/s: the DMP requires it */
	    wr(imu, RA_USER_CTRL, 0xC0) ||		/* FIFO + DMP enable */
	    wr(imu, RA_INT_ENABLE, 0x02) ||
	    wr(imu, RA_USER_CTRL, 0xC4))		/* reset the FIFO once, stay enabled */
		return -1;
	imu->gyro_lsb = DMP612_GYRO_LSB;
	printf("[balance] DMP 6.12 image loaded (%u bytes) - EXPERIMENTAL\n", (unsigned)len);
	return 0;
}

static int16_t be16(const uint8_t *p) { return (int16_t)(((uint16_t)p[0] << 8) | p[1]); }

int dmp612_parse(const uint8_t pkt[DMP612_PACKET_SIZE], dmp_sample_t *out)
{
	float n2;

	out->qw = (float)be16(pkt + 0) / Q_SCALE;
	out->qx = (float)be16(pkt + 4) / Q_SCALE;
	out->qy = (float)be16(pkt + 8) / Q_SCALE;
	out->qz = (float)be16(pkt + 12) / Q_SCALE;
	out->ax = be16(pkt + 16);
	out->ay = be16(pkt + 18);
	out->az = be16(pkt + 20);
	out->gx = be16(pkt + 22);
	out->gy = be16(pkt + 24);
	out->gz = be16(pkt + 26);
	n2 = out->qw * out->qw + out->qx * out->qx + out->qy * out->qy + out->qz * out->qz;
	return (n2 > 0.81f && n2 < 1.21f) ? 0 : -1;	/* |q| within 1 +- ~0.1 */
}

void dmp612_gravity(const dmp_sample_t *s, float g[3])
{
	g[0] = 2.0f * (s->qx * s->qz - s->qw * s->qy);
	g[1] = 2.0f * (s->qw * s->qx + s->qy * s->qz);
	g[2] = s->qw * s->qw - s->qx * s->qx - s->qy * s->qy + s->qz * s->qz;
}

static int fifo_reset(mpu60x0_t *imu)
{
	return wr(imu, RA_USER_CTRL, 0xC4);
}

int dmp612_poll(mpu60x0_t *imu, dmp_sample_t *out, dmp_stats_t *st)
{
	uint8_t cnt[2], pkt[DMP612_PACKET_SIZE];
	uint16_t count, n;
	int got = 0, i;

	if (mpu60x0_reg_read(imu, RA_FIFO_COUNTH, cnt, 2))
		return -1;
	count = (uint16_t)(((uint16_t)cnt[0] << 8) | cnt[1]);
	if (count >= DMP612_FIFO_SIZE - DMP612_PACKET_SIZE) {	/* hardware FIFO about to overwrite */
		fifo_reset(imu);
		st->overflows++;
		return -2;
	}
	n = (uint16_t)(count / DMP612_PACKET_SIZE);
	if (n == 0)
		return 1;
	if (n > DMP612_MAX_PACKETS) {			/* we are far behind: stale data is useless */
		fifo_reset(imu);
		st->overflows++;
		return -2;
	}
	for (i = 0; i < n; i++) {
		dmp_sample_t s;

		if (mpu60x0_reg_read(imu, RA_FIFO_RW, pkt, DMP612_PACKET_SIZE))
			return -1;
		st->packets++;
		if (dmp612_parse(pkt, &s)) {
			st->bad_packets++;
			continue;
		}
		*out = s;
		got = 1;
	}
	return got ? 0 : 1;
}
