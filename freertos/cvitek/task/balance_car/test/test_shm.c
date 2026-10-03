#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include "unity_lite.h"
#include "bc_shm.h"

static bc_shm_t g_shm_storage __attribute__((aligned(64)));
#define SHM ((volatile bc_shm_t *)&g_shm_storage)

static bc_telem_t mk(uint32_t n)
{
	bc_telem_t r;

	memset(&r, 0, sizeof(r));
	r.t_us = n * 5000u;
	r.pitch_cdeg = (int16_t)(n & 0x7FFF);
	r.gyro_y_cdps = (int16_t)(n & 0x7FFF);
	r.motor_l_pm = (int16_t)(n & 0x7FFF);
	r.enc_l = (int32_t)n;
	r.enc_r = (int32_t)~n;
	r.state = (uint8_t)(n & 0xFF);
	return r;
}

static int consistent(const bc_telem_t *r)
{
	uint32_t n = (uint32_t)r->enc_l;

	return r->enc_r == (int32_t)~n && r->pitch_cdeg == (int16_t)(n & 0x7FFF) &&
	       r->gyro_y_cdps == r->pitch_cdeg && r->motor_l_pm == r->pitch_cdeg &&
	       r->t_us == n * 5000u && r->state == (uint8_t)(n & 0xFF);
}

static void shm_init_writes_header_last(void)
{
	memset(&g_shm_storage, 0xAA, sizeof(g_shm_storage));
	bc_shm_init(SHM, 7, "abcdef0123456789-extra");
	CHECK_EQ(g_shm_storage.hdr.magic, BC_SHM_MAGIC);
	CHECK_EQ(g_shm_storage.hdr.version, BC_SHM_VERSION);
	CHECK_EQ(g_shm_storage.hdr.size, 0x10000);
	CHECK_EQ(g_shm_storage.hdr.boot_count, 7);
	CHECK(memcmp(g_shm_storage.hdr.build_id, "abcdef0123456789", 15) == 0);
	CHECK_EQ(g_shm_storage.hdr.build_id[15], 0);		/* truncated, terminated */
	CHECK_EQ(g_shm_storage.telem[3].seq, 0);		/* zeroed */
	CHECK_EQ(g_shm_storage.reserved[1000], 0);
}

static void shm_layout_offsets(void)
{
	CHECK_EQ(offsetof(bc_shm_t, status), 0x40);
	CHECK_EQ(offsetof(bc_shm_t, lnx), 0x80);
	CHECK_EQ(offsetof(bc_shm_t, param), 0x100);
	CHECK_EQ(offsetof(bc_shm_t, echo), 0x500);
	CHECK_EQ(offsetof(bc_shm_t, telem), 0x800);
	CHECK_EQ(offsetof(bc_shm_t, events), 0x4800);
	/* every Linux-written block starts on its own 64-byte cache line */
	CHECK_EQ(offsetof(bc_shm_t, lnx) % 64, 0);
	CHECK_EQ(offsetof(bc_shm_t, param) % 64, 0);
	CHECK(offsetof(bc_shm_t, lnx) >= offsetof(bc_shm_t, status) + 64);
}

static void shm_telem_roundtrip_and_order(void)
{
	bc_telem_t r = { 0 };
	uint32_t n;

	bc_shm_init(SHM, 1, "t");
	CHECK_EQ(bc_telem_read(SHM, 0, &r), 1);		/* nothing yet */
	for (n = 0; n < 10; n++) {
		bc_telem_t t = mk(n);

		bc_telem_push(SHM, &t);
	}
	CHECK_EQ(g_shm_storage.status.telem_head, 10);
	for (n = 0; n < 10; n++) {			/* SHM-02 */
		CHECK_EQ(bc_telem_read(SHM, n, &r), 0);
		CHECK(consistent(&r));
		CHECK_EQ(r.seq, 2 * n + 2);
	}
	CHECK_EQ(bc_telem_read(SHM, 10, &r), 1);
}

static void shm_telem_wraparound_reports_lost(void)
{
	bc_telem_t r = { 0 };
	uint32_t n;

	bc_shm_init(SHM, 1, "t");
	for (n = 0; n < 300; n++) {
		bc_telem_t t = mk(n);

		bc_telem_push(SHM, &t);
	}
	CHECK_EQ(bc_telem_read(SHM, 0, &r), -1);	/* overwritten */
	CHECK_EQ(bc_telem_read(SHM, 43, &r), -1);
	CHECK_EQ(bc_telem_read(SHM, 44, &r), 0);	/* oldest surviving */
	CHECK(consistent(&r));
	CHECK_EQ(bc_telem_read(SHM, 299, &r), 0);
}

static void shm_telem_detects_corruption_and_torn_slot(void)
{
	bc_telem_t r = { 0 }, t = mk(5);

	bc_shm_init(SHM, 1, "t");
	bc_telem_push(SHM, &t);
	g_shm_storage.telem[0].enc_l ^= 0x10;		/* bit flip */
	CHECK_EQ(bc_telem_read(SHM, 0, &r), -2);	/* crc */
	g_shm_storage.telem[0].enc_l ^= 0x10;
	CHECK_EQ(bc_telem_read(SHM, 0, &r), 0);
	g_shm_storage.telem[0].seq = 1u;		/* writer mid-update of record 0 */
	CHECK_EQ(bc_telem_read(SHM, 0, &r), -2);
}

static void shm_events_roundtrip_and_wrap(void)
{
	bc_event_t e = { 0 };
	uint32_t n;

	bc_shm_init(SHM, 1, "t");
	for (n = 0; n < 130; n++)
		bc_event_push(SHM, n, BC_EVT_LOG_STATE, (uint16_t)n, -(int32_t)n);
	CHECK_EQ(bc_event_read(SHM, 0, &e), -1);
	CHECK_EQ(bc_event_read(SHM, 2, &e), 0);
	CHECK_EQ(e.type, BC_EVT_LOG_STATE);
	CHECK_EQ(e.arg, 2);
	CHECK_EQ(e.val, -2);
	CHECK_EQ(bc_event_read(SHM, 129, &e), 0);
	CHECK_EQ(bc_event_read(SHM, 130, &e), 1);
}

static void shm_param_block_roundtrip(void)
{
	bc_params_t p, out = { 0 };
	uint32_t seq, s2 = 0;

	bc_shm_init(SHM, 1, "t");
	bc_params_default(&p);
	p.a_kp = 21.5f;
	seq = bc_param_blk_write(&g_shm_storage.param, &p);
	CHECK_EQ(seq, 2);
	CHECK_EQ(bc_param_blk_read(&g_shm_storage.param, &out, &s2), 0);
	CHECK_EQ(s2, 2);
	CHECK_NEAR(out.a_kp, 21.5, 1e-6);
	CHECK_EQ(bc_param_blk_write(&g_shm_storage.param, &p), 4);
}

static void shm_param_block_rejects_bad_data(void)
{
	bc_params_t p, out = { 0 };
	uint32_t seq = 0;

	bc_shm_init(SHM, 1, "t");
	bc_params_default(&p);
	bc_param_blk_write(&g_shm_storage.param, &p);

	g_shm_storage.param.seq |= 1u;			/* write in progress */
	CHECK_EQ(bc_param_blk_read(&g_shm_storage.param, &out, &seq), -1);
	g_shm_storage.param.seq &= ~1u;

	g_shm_storage.param.p.a_kd += 1.0f;		/* corruption */
	CHECK_EQ(bc_param_blk_read(&g_shm_storage.param, &out, &seq), -2);	/* SHM-03 */
	bc_param_blk_write(&g_shm_storage.param, &p);

	g_shm_storage.param.version = 99;		/* wrong layout version */
	CHECK_EQ(bc_param_blk_read(&g_shm_storage.param, &out, &seq), -2);
	bc_param_blk_write(&g_shm_storage.param, &p);

	CHECK_EQ(bc_param_blk_read(&g_shm_storage.param, &out, &seq), 0);
}

static void shm_param_echo_counts_applies(void)
{
	bc_params_t p;

	bc_shm_init(SHM, 1, "t");
	bc_params_default(&p);
	bc_param_echo_write(SHM, &p, 0, 4);
	bc_param_echo_write(SHM, &p, 3, 6);
	CHECK_EQ(g_shm_storage.echo.applied_count, 2);
	CHECK_EQ(g_shm_storage.echo.last_result, 3);
	CHECK_EQ(g_shm_storage.echo.applied_seq, 6);
	CHECK_EQ(g_shm_storage.echo.seq & 1u, 0);
}

/* --- SHM-01: seqlock stress with two threads ---------------------------- */
#define STRESS_N 300000u
static volatile int g_writer_done;

static void *telem_writer(void *arg)
{
	uint32_t n;

	(void)arg;
	for (n = 0; n < STRESS_N; n++) {
		bc_telem_t t = mk(n);

		bc_telem_push(SHM, &t);
		if ((n & 0x3FF) == 0)
			sched_yield();
	}
	g_writer_done = 1;
	return NULL;
}

static void shm_seqlock_stress_no_torn_reads(void)
{
	pthread_t th;
	unsigned ok = 0, torn = 0, lost = 0, bad = 0;
	uint32_t last = 0;

	bc_shm_init(SHM, 1, "stress");
	g_writer_done = 0;
	pthread_create(&th, NULL, telem_writer, NULL);
	while (!g_writer_done || last < STRESS_N) {
		uint32_t head = g_shm_storage.status.telem_head;
		bc_telem_t r = { 0 };
		int rc;

		if (head == 0)
			continue;
		rc = bc_telem_read(SHM, head - 1, &r);
		if (rc == 0) {
			ok++;
			if (!consistent(&r) || (uint32_t)r.enc_l != head - 1)
				bad++;
		} else if (rc == -1) {
			lost++;
		} else if (rc == -2) {
			torn++;			/* detected, not delivered */
		}
		last = head;
		if (g_writer_done && head >= STRESS_N)
			break;
	}
	pthread_join(th, NULL);
	CHECK(ok > 1000);
	CHECK_EQ(bad, 0);			/* never a torn record delivered */
	printf("    stress: ok=%u detected_torn=%u lost=%u\n", ok, torn, lost);
}

static void *param_writer(void *arg)
{
	bc_params_t p;
	uint32_t k;

	(void)arg;
	bc_params_default(&p);
	for (k = 1; k <= 50000; k++) {
		p.a_kp = (float)(k % 50) + 4.0f;
		p.a_kd = p.a_kp + 0.5f;		/* pair that must stay coupled */
		bc_param_blk_write(&g_shm_storage.param, &p);
		if ((k & 0xFF) == 0)
			sched_yield();
	}
	g_writer_done = 1;
	return NULL;
}

static void shm_param_stress_pairs_stay_coupled(void)
{
	pthread_t th;
	bc_params_t out = { 0 };
	uint32_t seq = 0;
	unsigned ok = 0, bad = 0;

	bc_shm_init(SHM, 1, "stress");
	g_writer_done = 0;
	pthread_create(&th, NULL, param_writer, NULL);
	while (!g_writer_done) {
		if (bc_param_blk_read(&g_shm_storage.param, &out, &seq) == 0) {
			ok++;
			if (fabsf(out.a_kd - (out.a_kp + 0.5f)) > 1e-5f)
				bad++;
		}
	}
	pthread_join(th, NULL);
	CHECK(ok > 100);
	CHECK_EQ(bad, 0);
}

void suite_shm(void)
{
	printf("suite shared memory\n");
	RUN(shm_init_writes_header_last);
	RUN(shm_layout_offsets);
	RUN(shm_telem_roundtrip_and_order);
	RUN(shm_telem_wraparound_reports_lost);
	RUN(shm_telem_detects_corruption_and_torn_slot);
	RUN(shm_events_roundtrip_and_wrap);
	RUN(shm_param_block_roundtrip);
	RUN(shm_param_block_rejects_bad_data);
	RUN(shm_param_echo_counts_applies);
	RUN(shm_seqlock_stress_no_torn_reads);
	RUN(shm_param_stress_pairs_stay_coupled);
}
