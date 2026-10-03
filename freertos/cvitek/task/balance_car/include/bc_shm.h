/*
 * Shared-memory window between the C906L (FreeRTOS) and Linux.
 * 64 KiB at the top of the 2 MiB FreeRTOS carve-out (linker symbol
 * _bc_shm_base, physical 0x9FFF0000 on a 512 MiB DuoS).
 *
 * Rules (docs/balance_car/01 section 6.3):
 *  - every 64-byte cache line has exactly ONE writer core;
 *  - RTOS never cleans/flushes a line Linux writes (it only invalidates
 *    before reading it), otherwise a write-back would clobber Linux data;
 *  - multi-word records use a seqlock (odd = being written).
 * Define BC_SHM_CLEAN / BC_SHM_INV before including to add cache maintenance
 * (RTOS build); they default to no-ops (host tests, Linux non-cacheable map).
 */
#ifndef BC_SHM_H
#define BC_SHM_H

#include <stdint.h>
#include <string.h>
#include "bc_params.h"

#ifndef BC_SHM_CLEAN
#define BC_SHM_CLEAN(p, n)	((void)(p), (void)(n))
#endif
#ifndef BC_SHM_INV
#define BC_SHM_INV(p, n)	((void)(p), (void)(n))
#endif
#define BC_FENCE()		__atomic_thread_fence(__ATOMIC_SEQ_CST)

#define BC_SHM_MAGIC		0x4D534342u	/* 'BCSM' */
#define BC_SHM_VERSION		1u
#define BC_SHM_SIZE		0x10000u
#define BC_SHM_PHYS_DEFAULT	0x9FFF0000u

#define BC_TELEM_SLOTS		256u
#define BC_EVENT_SLOTS		128u

/* --- RTOS-written, cache line 0 ----------------------------------------- */
typedef struct {
	uint32_t magic, version, size, boot_count;
	char build_id[16];
	uint32_t reserved[8];
} bc_shm_hdr_t;				/* 64 B */

/* --- RTOS-written, cache line 1 ----------------------------------------- */
typedef struct {
	uint32_t rtos_heartbeat;	/* ++ at 10 Hz                           */
	uint32_t state, fault;
	uint32_t deadline_miss, imu_err, cpu_load_pct;
	uint32_t telem_head;		/* absolute count of records written     */
	uint32_t event_head;
	uint32_t cycles;		/* control cycles executed               */
	uint32_t lost_cmds;
	uint32_t stack_min_words[4];
	uint32_t imu_variant;		/* mpu_variant_t: 0 unknown, 1 MPU6050, 2 MPU6500 family */
	uint32_t dmp_info;		/* bits 0-7 BC_DMP_*, bits 8-31 DMP->filter fallbacks (saturating) */
} bc_shm_status_t;			/* 64 B */

/* --- Linux-written, its own cache line ---------------------------------- */
typedef struct {
	uint32_t linux_heartbeat;
	uint32_t linux_flags;
	uint32_t reserved[14];
} bc_shm_linux_t;			/* 64 B */

/* --- Linux-written parameter block (seqlock) ---------------------------- */
typedef struct {
	uint32_t seq;
	uint32_t crc;			/* crc32 over version..p                 */
	uint32_t version;
	uint32_t size;			/* sizeof(bc_params_t)                   */
	bc_params_t p;
} bc_param_blk_t;

/* --- RTOS-written: what is actually in use ------------------------------ */
typedef struct {
	uint32_t seq;
	uint32_t applied_count;
	uint32_t last_result;		/* 0 ok, else 1 + bad field index        */
	uint32_t applied_seq;		/* param block seq that was applied      */
	bc_params_t p;
} bc_param_echo_t;

/* --- Linux-written DMP firmware image (experimental est_mode 2) ---------- */
#define BC_DMP_KIND_612		612	/* InvenSense MotionApps 6.12, 3062 bytes */
#define BC_DMP_DATA_MAX		4080
typedef struct {
	uint32_t seq;			/* seqlock, Linux is the only writer      */
	uint32_t crc;			/* crc32 over kind, size, data[size]      */
	uint32_t kind;
	uint32_t size;
	uint8_t data[BC_DMP_DATA_MAX];
} bc_dmp_blk_t;				/* 4096 B */

/* status.dmp_info low byte */
#define BC_DMP_OFF		0	/* not requested / raw-data path            */
#define BC_DMP_RUNNING		1
#define BC_DMP_LOAD_FAILED	2	/* image write/verify failed, fell back     */
#define BC_DMP_NO_IMAGE		3	/* est_mode 2 requested but no valid image  */
#define BC_DMP_LOST		4	/* FIFO stopped delivering packets          */

/* --- RTOS-written calibration ------------------------------------------- */
typedef struct {
	uint32_t seq;
	uint32_t valid;
	float gyro_bias[3];		/* deg/s */
	float accel_mean[3];		/* g     */
	float trim_deg;
	float temp_c;
	uint32_t reserved[8];
} bc_calib_t;

/* --- telemetry record, 64 B --------------------------------------------- */
typedef struct {
	uint32_t seq;			/* 2*n+2 when record n is stable          */
	uint32_t t_us;
	int16_t pitch_cdeg, pitch_acc_cdeg, gyro_y_cdps, gyro_z_cdps;
	int16_t accel_norm_mg, speed_l_mmps, speed_r_mmps, angle_set_cdeg;
	int16_t motor_l_pm, motor_r_pm, target_speed_mmps, target_turn_mradps;
	int32_t enc_l, enc_r;
	uint16_t vbat_mv, exec_us, period_us;
	uint8_t state, fault, imu_err, flags;
	int16_t psi_mrad;		/* heading from the yaw gyro [mrad]       */
	int32_t x_mm;			/* odometry distance [mm]                 */
	uint8_t pad[4];
	uint32_t crc;			/* crc32 over bytes 4..59                 */
} bc_telem_t;

typedef struct {
	uint32_t seq;
	uint32_t t_us;
	uint16_t type, arg;
	int32_t val;
	uint8_t pad[16];
} bc_event_t;				/* 32 B */

#define BC_EVT_LOG_STATE	1	/* arg = state, val = fault              */
#define BC_EVT_LOG_CMD		2	/* arg = cmd id, val = param             */
#define BC_EVT_LOG_PARAMS	3	/* arg = result, val = block seq         */
#define BC_EVT_LOG_CALIB	4	/* arg = kind, val = result              */
#define BC_EVT_LOG_TIMING	5	/* arg = 0, val = period_us              */
#define BC_EVT_LOG_PONG		6	/* val = nonce of the PING               */

typedef struct {
	bc_shm_hdr_t hdr;				/* 0x0000 */
	bc_shm_status_t status;				/* 0x0040 */
	bc_shm_linux_t lnx;				/* 0x0080 */
	uint8_t pad0[0x100 - 0xC0];
	bc_param_blk_t param;				/* 0x0100 */
	uint8_t pad1[0x500 - 0x100 - sizeof(bc_param_blk_t)];
	bc_param_echo_t echo;				/* 0x0500 */
	uint8_t pad2[0x600 - 0x500 - sizeof(bc_param_echo_t)];
	bc_calib_t calib;				/* 0x0600 */
	uint8_t pad3[0x800 - 0x600 - sizeof(bc_calib_t)];
	bc_telem_t telem[BC_TELEM_SLOTS];		/* 0x0800 */
	bc_event_t events[BC_EVENT_SLOTS];		/* 0x4800 */
	uint8_t pad4[0x6000 - 0x5800];
	bc_dmp_blk_t dmp;				/* 0x6000 */
	uint8_t reserved[BC_SHM_SIZE - 0x7000];
} bc_shm_t;

_Static_assert(sizeof(bc_shm_hdr_t) == 64, "hdr");
_Static_assert(sizeof(bc_shm_status_t) == 64, "status");
_Static_assert(sizeof(bc_shm_linux_t) == 64, "linux");
_Static_assert(sizeof(bc_telem_t) == 64, "telem");
_Static_assert(sizeof(bc_event_t) == 32, "event");
_Static_assert(sizeof(bc_param_blk_t) <= 0x400, "param blk");
_Static_assert(sizeof(bc_param_echo_t) <= 0x100, "param echo");
_Static_assert(sizeof(bc_calib_t) <= 0x200, "calib");
_Static_assert(sizeof(bc_shm_t) == BC_SHM_SIZE, "shm size");
_Static_assert(offsetof(bc_shm_t, status) == 0x40, "status off");
_Static_assert(offsetof(bc_shm_t, lnx) == 0x80, "linux off");
_Static_assert(offsetof(bc_shm_t, param) == 0x100, "param off");
_Static_assert(offsetof(bc_shm_t, echo) == 0x500, "echo off");
_Static_assert(offsetof(bc_shm_t, calib) == 0x600, "calib off");
_Static_assert(offsetof(bc_shm_t, telem) == 0x800, "telem off");
_Static_assert(offsetof(bc_shm_t, events) == 0x4800, "events off");
_Static_assert(sizeof(bc_dmp_blk_t) == 0x1000, "dmp blk");
_Static_assert(offsetof(bc_shm_t, dmp) == 0x6000, "dmp off");

/* ---- RTOS side ----------------------------------------------------------- */

/* Zero the window and write the header. */
static inline void bc_shm_init(volatile bc_shm_t *s, uint32_t boot_count,
			       const char *build_id)
{
	volatile uint8_t *b = (volatile uint8_t *)s;
	size_t i;

	for (i = 0; i < BC_SHM_SIZE; i++)
		b[i] = 0;
	s->hdr.version = BC_SHM_VERSION;
	s->hdr.size = BC_SHM_SIZE;
	s->hdr.boot_count = boot_count;
	for (i = 0; i < sizeof(s->hdr.build_id) - 1 && build_id && build_id[i]; i++)
		s->hdr.build_id[i] = build_id[i];
	BC_FENCE();
	s->hdr.magic = BC_SHM_MAGIC;	/* last: readers wait for the magic */
	BC_SHM_CLEAN(s, BC_SHM_SIZE);
}

static inline void bc_telem_push(volatile bc_shm_t *s, const bc_telem_t *rec)
{
	uint32_t n = s->status.telem_head;
	volatile bc_telem_t *slot = &s->telem[n % BC_TELEM_SLOTS];
	bc_telem_t tmp = *rec;

	tmp.seq = 2u * n + 2u;
	tmp.crc = bc_crc32((const uint8_t *)&tmp + 4, 56);
	slot->seq = 2u * n + 1u;		/* odd: being written */
	BC_FENCE();
	memcpy((void *)((uint8_t *)slot + 4), (const uint8_t *)&tmp + 4, 60);
	BC_FENCE();
	slot->seq = tmp.seq;
	BC_SHM_CLEAN(slot, sizeof(*slot));
	s->status.telem_head = n + 1u;
	BC_SHM_CLEAN(&s->status, sizeof(s->status));
}

static inline void bc_event_push(volatile bc_shm_t *s, uint32_t t_us,
				 uint16_t type, uint16_t arg, int32_t val)
{
	uint32_t n = s->status.event_head;
	volatile bc_event_t *slot = &s->events[n % BC_EVENT_SLOTS];

	slot->seq = 2u * n + 1u;
	BC_FENCE();
	slot->t_us = t_us;
	slot->type = type;
	slot->arg = arg;
	slot->val = val;
	BC_FENCE();
	slot->seq = 2u * n + 2u;
	BC_SHM_CLEAN(slot, sizeof(*slot));
	s->status.event_head = n + 1u;
	BC_SHM_CLEAN(&s->status, sizeof(s->status));
}

/* Read the Linux-written parameter block. 0 ok, -1 torn/in progress, -2 bad
 * size/version/crc. *seq_out receives the (even) sequence number. */
static inline int bc_param_blk_read(volatile bc_param_blk_t *b,
				    bc_params_t *out, uint32_t *seq_out)
{
	bc_param_blk_t tmp;
	uint32_t s1, s2;

	BC_SHM_INV(b, sizeof(*b));
	s1 = b->seq;
	if (s1 & 1u)
		return -1;
	BC_FENCE();
	memcpy(&tmp, (const void *)b, sizeof(tmp));
	BC_FENCE();
	s2 = b->seq;
	if (s1 != s2)
		return -1;
	if (tmp.version != BC_SHM_VERSION || tmp.size != sizeof(bc_params_t))
		return -2;
	if (tmp.crc != bc_crc32(&tmp.version, 8 + sizeof(bc_params_t)))
		return -2;
	*out = tmp.p;
	*seq_out = s1;
	return 0;
}

static inline void bc_param_echo_write(volatile bc_shm_t *s,
				       const bc_params_t *p, uint32_t result,
				       uint32_t applied_seq)
{
	volatile bc_param_echo_t *e = &s->echo;
	uint32_t seq = e->seq;

	e->seq = seq | 1u;
	BC_FENCE();
	memcpy((void *)&e->p, p, sizeof(*p));
	e->last_result = result;
	e->applied_seq = applied_seq;
	e->applied_count++;
	BC_FENCE();
	e->seq = (seq | 1u) + 1u;
	BC_SHM_CLEAN(e, sizeof(*e));
}

/* Publish the calibration block (seqlock; RTOS is the only writer). */
static inline void bc_calib_publish(volatile bc_shm_t *s, const float gyro_bias[3],
				    const float accel_mean[3], float trim_deg, float temp_c)
{
	volatile bc_calib_t *c = &s->calib;
	uint32_t seq = c->seq;
	int i;

	c->seq = seq | 1u;
	BC_FENCE();
	for (i = 0; i < 3; i++) {
		c->gyro_bias[i] = gyro_bias[i];
		c->accel_mean[i] = accel_mean[i];
	}
	c->trim_deg = trim_deg;
	c->temp_c = temp_c;
	c->valid = 1;
	BC_FENCE();
	c->seq = (seq | 1u) + 1u;
	BC_SHM_CLEAN(c, sizeof(*c));
}

/* ---- Linux side (also used by host tests) ------------------------------- */

/* Writer for the parameter block. Returns the new even sequence number. */
static inline uint32_t bc_param_blk_write(volatile bc_param_blk_t *b,
					  const bc_params_t *p)
{
	uint32_t seq = b->seq;
	bc_param_blk_t tmp;

	memset(&tmp, 0, sizeof(tmp));
	tmp.version = BC_SHM_VERSION;
	tmp.size = sizeof(bc_params_t);
	tmp.p = *p;
	tmp.crc = bc_crc32(&tmp.version, 8 + sizeof(bc_params_t));
	seq = (seq | 1u);
	b->seq = seq;				/* odd: in progress */
	BC_FENCE();
	memcpy((void *)((uint8_t *)b + 4), (const uint8_t *)&tmp + 4,
	       sizeof(tmp) - 4);
	BC_FENCE();
	b->seq = seq + 1u;
	return seq + 1u;
}

/* Writer for the DMP image block. Returns the new even sequence number, 0 if too big. */
static inline uint32_t bc_dmp_blk_write(volatile bc_dmp_blk_t *b, uint32_t kind,
					const uint8_t *data, uint32_t size)
{
	uint32_t seq = b->seq;

	if (size == 0 || size > BC_DMP_DATA_MAX)
		return 0;
	seq |= 1u;
	b->seq = seq;				/* odd: in progress */
	BC_FENCE();
	memcpy((void *)b->data, data, size);
	b->kind = kind;
	b->size = size;
	b->crc = bc_crc32((const void *)&b->kind, 8 + size);	/* kind, size, data are contiguous */
	BC_FENCE();
	b->seq = seq + 1u;
	return seq + 1u;
}

/* RTOS side: copy the image into out[cap]. 0 ok (size in *size), -1 absent/torn, -2 invalid. */
static inline int bc_dmp_blk_read(volatile bc_dmp_blk_t *b, uint32_t kind,
				  uint8_t *out, uint32_t cap, uint32_t *size)
{
	uint32_t s1, s2, sz, crc;

	BC_SHM_INV(b, sizeof(*b));
	s1 = b->seq;
	if (s1 == 0 || (s1 & 1u))
		return -1;
	BC_FENCE();
	sz = b->size;
	if (b->kind != kind || sz == 0 || sz > BC_DMP_DATA_MAX || sz > cap)
		return -2;
	memcpy(out, (const void *)b->data, sz);
	crc = b->crc;
	BC_FENCE();
	s2 = b->seq;
	if (s1 != s2)
		return -1;
	if (crc != bc_crc32((const void *)&b->kind, 8 + sz))
		return -2;
	*size = sz;
	return 0;
}

/* Telemetry reader for absolute record index n.
 * 0 ok, 1 not written yet, -1 overwritten (lost), -2 torn / crc mismatch. */
static inline int bc_telem_read(volatile bc_shm_t *s, uint32_t n, bc_telem_t *out)
{
	volatile bc_telem_t *slot = &s->telem[n % BC_TELEM_SLOTS];
	uint32_t head = s->status.telem_head, want = 2u * n + 2u, s1, s2;

	if (n >= head)
		return 1;
	if (head - n > BC_TELEM_SLOTS)
		return -1;
	s1 = slot->seq;
	if (s1 > want)
		return -1;
	if (s1 != want)
		return -2;
	BC_FENCE();
	memcpy(out, (const void *)slot, sizeof(*out));
	BC_FENCE();
	s2 = slot->seq;
	if (s2 != want)
		return (s2 > want) ? -1 : -2;
	if (out->crc != bc_crc32((const uint8_t *)out + 4, 56))
		return -2;
	return 0;
}

static inline int bc_event_read(volatile bc_shm_t *s, uint32_t n, bc_event_t *out)
{
	volatile bc_event_t *slot = &s->events[n % BC_EVENT_SLOTS];
	uint32_t head = s->status.event_head, want = 2u * n + 2u, s1, s2;

	if (n >= head)
		return 1;
	if (head - n > BC_EVENT_SLOTS)
		return -1;
	s1 = slot->seq;
	if (s1 > want)
		return -1;
	if (s1 != want)
		return -2;
	BC_FENCE();
	memcpy(out, (const void *)slot, sizeof(*out));
	BC_FENCE();
	s2 = slot->seq;
	if (s2 != want)
		return (s2 > want) ? -1 : -2;
	return 0;
}

#endif /* BC_SHM_H */
