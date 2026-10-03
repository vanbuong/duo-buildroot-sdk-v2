#include <string.h>
#include "unity_lite.h"
#include "imu_health.h"
#include "bc_cmd.h"
#include "bc_proto.h"

static void health_stuck_frame_detector(void)
{
	bc_imu_health_t h;
	uint8_t f[14] = { 1, 2, 3 };
	int i;

	bc_imu_health_init(&h);
	for (i = 0; i < 20; i++)
		bc_imu_health_update(&h, 1, f, 0, 0, 0, 1.0f, 5.0f);
	CHECK(!h.stuck);				/* 19 repeats */
	bc_imu_health_update(&h, 1, f, 0, 0, 0, 1.0f, 5.0f);
	CHECK(h.stuck);					/* FLT-01: 20 repeats */
	f[13] ^= 1;
	bc_imu_health_update(&h, 1, f, 0, 0, 0, 1.0f, 5.0f);
	CHECK(!h.stuck);				/* one differing byte clears */
}

static void health_gyro_saturation(void)
{
	bc_imu_health_t h;
	int i;

	bc_imu_health_init(&h);
	for (i = 0; i < 3; i++)
		bc_imu_health_update(&h, 1, NULL, 0, 32767, 0, 1.0f, 5.0f);
	CHECK(!h.range);				/* FLT-02: 3 cycles ok */
	bc_imu_health_update(&h, 1, NULL, 0, 32767, 0, 1.0f, 5.0f);
	CHECK(h.range);
}

static void health_gyro_saturation_resets_when_clear(void)
{
	bc_imu_health_t h;
	int i;

	bc_imu_health_init(&h);
	for (i = 0; i < 100; i++) {
		bc_imu_health_update(&h, 1, NULL, 0, 32767, 0, 1.0f, 5.0f);
		bc_imu_health_update(&h, 1, NULL, 0, 0, 0, 1.0f, 5.0f);
	}
	CHECK(!h.range);
}

static void health_accel_norm_window(void)
{
	bc_imu_health_t h;
	int i;

	bc_imu_health_init(&h);
	for (i = 0; i < 19; i++)
		bc_imu_health_update(&h, 1, NULL, 0, 0, 0, 0.4f, 5.0f);
	CHECK(!h.norm_warn);				/* 95 ms */
	bc_imu_health_update(&h, 1, NULL, 0, 0, 0, 0.4f, 10.0f);
	CHECK(h.norm_warn);				/* FLT-03: >100 ms */
	CHECK(!h.range);
	for (i = 0; i < 80; i++)
		bc_imu_health_update(&h, 1, NULL, 0, 0, 0, 0.4f, 5.0f);
	CHECK(h.range);					/* >500 ms */
	bc_imu_health_update(&h, 1, NULL, 0, 0, 0, 1.0f, 5.0f);
	CHECK(!h.norm_warn);
}

static void health_bus_error_policy(void)
{
	bc_imu_health_t h;

	bc_imu_health_init(&h);
	bc_imu_health_update(&h, 0, NULL, 0, 0, 0, 1.0f, 5.0f);
	bc_imu_health_update(&h, 0, NULL, 0, 0, 0, 1.0f, 5.0f);
	CHECK(!h.bus_fail);				/* FLT-04: 1-2 reuse */
	bc_imu_health_update(&h, 1, NULL, 0, 0, 0, 1.0f, 5.0f);
	bc_imu_health_update(&h, 0, NULL, 0, 0, 0, 1.0f, 5.0f);
	bc_imu_health_update(&h, 0, NULL, 0, 0, 0, 1.0f, 5.0f);
	CHECK(!h.bus_fail);				/* streak was reset */
	bc_imu_health_update(&h, 0, NULL, 0, 0, 0, 1.0f, 5.0f);
	CHECK(h.bus_fail);				/* 3 in a row */
	CHECK_EQ(h.total_errors, 5);
	bc_imu_health_update(&h, 1, NULL, 0, 0, 0, 1.0f, 5.0f);
	CHECK(!h.bus_fail);
}

static void cmd_pack_unpack_roundtrip(void)
{
	uint32_t p = bc_pack_target(-1234, 4321);

	CHECK_EQ(bc_unpack_speed(p), -1234);		/* CMD-01 */
	CHECK_EQ(bc_unpack_turn(p), 4321);
	p = bc_pack_target(100000, -100000);		/* clamps to int16 */
	CHECK_EQ(bc_unpack_speed(p), 32767);
	CHECK_EQ(bc_unpack_turn(p), -32768);
	p = bc_pack_target(0, 0);
	CHECK_EQ(p, 0);
}

static void proto_state_pack(void)
{
	uint32_t p = bc_pack_state(4, 6, 0xBEEF);

	CHECK_EQ(bc_unpack_state(p), 4);
	CHECK_EQ(bc_unpack_fault(p), 6);
	CHECK_EQ(bc_unpack_flags(p), 0xBEEF);
}

static void cmd_targets_clamped_to_limits(void)
{
	bc_params_t p;
	bc_cmd_t c;
	float v, w;
	int z, lost;

	bc_params_default(&p);
	bc_cmd_init(&c, 1000);
	bc_cmd_heartbeat(&c, 1, 1000);
	bc_cmd_set_target(&c, 5000, -9000, 1000);
	bc_cmd_effective(&c, &p, 1010, &v, &w, &z, &lost);
	CHECK_NEAR(v, p.v_max, 1e-6);
	CHECK_NEAR(w, -p.w_max, 1e-6);
	CHECK(!z && !lost);
}

static void cmd_watchdog_zeroes_targets_not_disarm(void)
{
	bc_params_t p;
	bc_cmd_t c;
	float v, w;
	int z, lost;

	bc_params_default(&p);
	bc_cmd_init(&c, 0);
	bc_cmd_heartbeat(&c, 1, 0);
	bc_cmd_set_target(&c, 300, 500, 0);
	bc_cmd_effective(&c, &p, 400, &v, &w, &z, &lost);
	CHECK(v > 0.29f && !z);
	/* new targets keep arriving but the heartbeat counter froze */
	bc_cmd_set_target(&c, 300, 500, 700);
	bc_cmd_heartbeat(&c, 1, 700);			/* same counter: no refresh */
	bc_cmd_effective(&c, &p, 800, &v, &w, &z, &lost);
	CHECK(z);					/* CMD-02: 800 > 500 ms */
	CHECK_NEAR(v, 0.0, 1e-9);
	CHECK_NEAR(w, 0.0, 1e-9);
	CHECK(!lost);					/* hb_disarm_ms = 0: never */
	bc_cmd_heartbeat(&c, 2, 810);
	bc_cmd_set_target(&c, 300, 0, 810);
	bc_cmd_effective(&c, &p, 820, &v, &w, &z, &lost);
	CHECK(!z && v > 0.29f);				/* resumes */
}

static void cmd_stale_target_zeroed_even_with_heartbeat(void)
{
	bc_params_t p;
	bc_cmd_t c;
	float v, w;
	int z, lost;

	bc_params_default(&p);
	bc_cmd_init(&c, 0);
	bc_cmd_set_target(&c, 300, 0, 0);
	bc_cmd_heartbeat(&c, 1, 0);
	bc_cmd_heartbeat(&c, 2, 600);
	bc_cmd_effective(&c, &p, 700, &v, &w, &z, &lost);
	CHECK(z);
	CHECK_NEAR(v, 0.0, 1e-9);
}

static void cmd_hb_disarm_threshold_and_wraparound(void)
{
	bc_params_t p;
	bc_cmd_t c;
	float v, w;
	int z, lost;

	bc_params_default(&p);
	p.hb_disarm_ms = 5000.0f;
	bc_cmd_init(&c, 0xFFFFFF00u);
	bc_cmd_heartbeat(&c, 1, 0xFFFFFF00u);
	bc_cmd_effective(&c, &p, 0x00000100u, &v, &w, &z, &lost);	/* 512 ms, wrapped */
	CHECK(!lost);
	bc_cmd_effective(&c, &p, 0x00001900u, &v, &w, &z, &lost);	/* > 5 s */
	CHECK(lost);
}

void suite_health_cmd(void)
{
	printf("suite imu health / command / protocol\n");
	RUN(health_stuck_frame_detector);
	RUN(health_gyro_saturation);
	RUN(health_gyro_saturation_resets_when_clear);
	RUN(health_accel_norm_window);
	RUN(health_bus_error_policy);
	RUN(cmd_pack_unpack_roundtrip);
	RUN(proto_state_pack);
	RUN(cmd_targets_clamped_to_limits);
	RUN(cmd_watchdog_zeroes_targets_not_disarm);
	RUN(cmd_stale_target_zeroed_even_with_heartbeat);
	RUN(cmd_hb_disarm_threshold_and_wraparound);
}
