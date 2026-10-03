/*
 * Closed-loop tests: the real bc_core (estimator + state machine +
 * controller) and the real MPU driver run against a simulated wheeled
 * inverted pendulum (sim_plant.c). The IMU values travel through the fake
 * I2C register file, so frame decoding, scaling and the health monitor are
 * exercised too. The loop mirrors balance_ctrl_task() in src/balance_main.c.
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "unity_lite.h"
#include "bc_core.h"
#include "mpu60x0.h"
#include "fake_hw.h"
#include "sim_plant.h"

#define RAD2DEG 57.2957795
#define DT 0.005

typedef struct {
	bc_params_t p;
	double ku;			/* plant gain override                */
	double theta0_deg;		/* lean at the moment the car is released */
	double accel_noise_g;
	double gyro_noise_dps;
	double push_at_s, push_rad_s;	/* impulse on pitch rate, after release */
	double unplug_at_s;		/* IMU stops answering, <0 = never    */
	double seconds;			/* simulated time after release       */
	int dmp;			/* 0 none, 1 true angle, 2 frozen at 0 deg (broken DMP) */
} sim_cfg_t;

typedef struct {
	double max_theta_deg;
	double tail_theta_deg;		/* max |theta| over the last second   */
	double max_x_m;
	unsigned dmp_fallbacks;		/* est_mode 2: cycles that used the filter */
	int dmp_used_cycles;
	double final_x_m;		/* signed position at the end         */
	double tail_speed_mps;
	int fell;			/* core went to FALLEN/FAULT after release */
	int released;			/* reached BALANCING                  */
	bc_state_id_t final_state;
	bc_fault_t final_fault;
	int cycles_to_fault;		/* after unplug; -1 if no fault       */
	double motor_after_fault;	/* max |motor| after a fault          */
	int seq_ok;			/* CALIBRATING->IDLE->ARMING->BALANCING seen in order */
	double arming_s;
} sim_res_t;

static uint32_t g_rng = 12345;
static double urand(void)
{
	g_rng = g_rng * 1664525u + 1013904223u;
	return ((g_rng >> 8) & 0xFFFF) / 32768.0 - 1.0;
}

static int16_t sat16(double v)
{
	return (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
}

static sim_cfg_t cfg_default(void)
{
	sim_cfg_t c;

	memset(&c, 0, sizeof(c));
	bc_params_default(&c.p);
	c.ku = 0.05; c.theta0_deg = 5.0; c.seconds = 15.0;
	/* baseline MPU noise (a perfectly noiseless sensor would trip the
	 * stuck-frame detector, rightly) */
	c.accel_noise_g = 0.004; c.gyro_noise_dps = 0.05;
	c.push_at_s = -1.0; c.unplug_at_s = -1.0;
	return c;
}

static sim_res_t run_sim(const sim_cfg_t *c)
{
	bc_core_t core;
	bc_core_in_t in;
	bc_core_out_t out;
	mpu60x0_t imu;
	plant_t pl;
	sim_res_t r;
	long i, n_after = 0, total = 0;
	double t_rel = -1, prev_cnt = 0;
	int pushed = 0, unplugged = 0, fault_cycles = -1;
	int saw_calib = 0, saw_idle = 0, saw_arming = 0, saw_bal = 0, order_ok = 1;
	bc_state_id_t prev_state = BC_ST_BOOT;

	memset(&r, 0, sizeof(r));
	memset(&in, 0, sizeof(in));
	memset(&out, 0, sizeof(out));
	memset(&imu, 0, sizeof(imu));
	r.cycles_to_fault = -1;
	g_rng = 12345;
	fake_i2c_reset();
	plant_init(&pl);
	pl.ku = c->ku;
	bc_core_init(&core, &c->p);
	(void)prev_cnt;

	for (i = 0; ; i++) {
		double t = i * DT, th = pl.theta, s = sin(th), co = cos(th);
		double lin_a_g = pl.ku * pl.u_applied / 9.81;
		mpu60x0_scaled_t sc;
		int ok;
		double t_after = (t_rel < 0) ? -1 : t - t_rel;

		if (t_rel >= 0 && t_after >= c->seconds)
			break;
		if (i > 200000)
			break;
		if (t_rel < 0 && i > 4000)
			break;			/* never armed */

		if (t_rel >= 0 && !pushed && c->push_at_s >= 0 && t_after >= c->push_at_s) {
			pl.omega += c->push_rad_s;
			pushed = 1;
		}
		if (t_rel >= 0 && !unplugged && c->unplug_at_s >= 0 && t_after >= c->unplug_at_s) {
			fake_i2c_fail_reads(1);
			unplugged = 1;
			fault_cycles = 0;
		}

		/* what the IMU measures (accel includes the wheel acceleration) */
		fake_i2c_set_accel_gyro(
			sat16((-s + lin_a_g * co + c->accel_noise_g * urand()) * MPU_ACCEL_LSB_PER_G),
			sat16(c->accel_noise_g * urand() * MPU_ACCEL_LSB_PER_G),
			sat16((co + lin_a_g * s + c->accel_noise_g * urand()) * MPU_ACCEL_LSB_PER_G),
			0,
			sat16((pl.omega * RAD2DEG + c->gyro_noise_dps * urand()) * MPU_GYRO_LSB_PER_DPS),
			0);
		ok = (mpu60x0_read(&imu) == 0);
		mpu60x0_scale(&imu, &sc);

		in.imu_ok = ok;
		in.raw = imu.raw;
		in.ax = sc.ax; in.ay = sc.ay; in.az = sc.az;
		in.gy = sc.gy; in.gz = sc.gz;
		in.raw_gx = imu.gx; in.raw_gy = imu.gy; in.raw_gz = imu.gz;
		if (c->dmp) {
			in.dmp_valid = 1;
			in.dmp_g[0] = c->dmp == 1 ? (float)-s : 0.0f;
			in.dmp_g[1] = 0.0f;
			in.dmp_g[2] = c->dmp == 1 ? (float)co : 1.0f;
		}
		in.enc_l = in.enc_r = (int32_t)plant_encoder_counts(&pl);
		in.v_target = in.w_target = 0.0f;
		in.dt = (float)DT;
		in.timing_fault = 0;
		in.events = 0;
		if (i == 0) in.events = BC_EV_CALIB_REQ;
		if (i == 1) in.events = BC_EV_CALIB_OK;
		if (i == 2) in.events = BC_EV_ARM;
		bc_core_step(&core, &in, &out);
		if (out.dmp_used)
			r.dmp_used_cycles++;

		/* state sequence bookkeeping */
		if (out.state != prev_state) {
			if (out.state == BC_ST_CALIBRATING) saw_calib = 1;
			if (out.state == BC_ST_IDLE) { if (!saw_calib) order_ok = 0; saw_idle = 1; }
			if (out.state == BC_ST_ARMING) { if (!saw_idle) order_ok = 0; saw_arming = 1; r.arming_s = 0; }
			if (out.state == BC_ST_BALANCING && t_rel < 0) {
				if (!saw_arming) order_ok = 0;
				saw_bal = 1;
				t_rel = t;	/* "hand release" */
				pl.theta = c->theta0_deg / RAD2DEG;
				pl.omega = 0;
				/* the car was held at this lean: estimator has converged to it */
				core.est.theta = (float)c->theta0_deg;
			}
			prev_state = out.state;
		}
		if (out.state == BC_ST_ARMING)
			r.arming_s += DT;

		if (t_rel < 0) {
			/* held upright by hand while arming */
			pl.theta = pl.omega = pl.x = pl.v = pl.u_applied = 0.0;
			continue;
		}

		{
			double drive = out.motor_enable ? 0.5 * (out.out_l + out.out_r) : 0.0;

			if (out.state == BC_ST_FALLEN || out.state == BC_ST_FAULT) {
				r.fell = 1;
				if (fault_cycles >= 0 && r.cycles_to_fault < 0)
					r.cycles_to_fault = fault_cycles;
				if (fabs(drive) > r.motor_after_fault)
					r.motor_after_fault = fabs(drive);
			}
			if (fault_cycles >= 0)
				fault_cycles++;
			plant_step(&pl, drive, DT);
		}

		total++;
		n_after++;
		if (fabs(plant_theta_deg(&pl)) > r.max_theta_deg)
			r.max_theta_deg = fabs(plant_theta_deg(&pl));
		if (fabs(pl.x) > r.max_x_m)
			r.max_x_m = fabs(pl.x);
		if (t - t_rel >= c->seconds - 1.0) {
			if (fabs(plant_theta_deg(&pl)) > r.tail_theta_deg)
				r.tail_theta_deg = fabs(plant_theta_deg(&pl));
			if (fabs(pl.v) > r.tail_speed_mps)
				r.tail_speed_mps = fabs(pl.v);
		}
		if (fabs(pl.theta) > 3.0 && !unplugged) {	/* diverged */
			r.max_theta_deg = 1e6;
			break;
		}
	}
	(void)n_after; (void)total;
	r.final_x_m = pl.x;
	r.dmp_fallbacks = core.est.dmp_fallbacks;
	r.released = saw_bal;
	r.seq_ok = order_ok && saw_calib && saw_idle && saw_arming && saw_bal;
	r.final_state = out.state;
	r.final_fault = out.fault;
	return r;
}

/* ---- tests ------------------------------------------------------------- */

static void sim_startup_sequence_and_arming_time(void)
{
	sim_cfg_t c = cfg_default();
	sim_res_t r;

	c.seconds = 3.0;
	r = run_sim(&c);
	CHECK(r.released);
	CHECK(r.seq_ok);				/* CALIBRATING->IDLE->ARMING->BALANCING */
	CHECK_NEAR(r.arming_s, 1.0, 0.05);		/* ST-02: 1 s upright */
	CHECK(!r.fell);
}

static void sim_default_params_balance_across_plant_gains(void)
{
	const double kus[] = { 0.02, 0.05, 0.1, 0.2 };	/* 0.4x .. 4x assumed gain */
	size_t i;

	for (i = 0; i < sizeof(kus) / sizeof(kus[0]); i++) {
		sim_cfg_t c = cfg_default();
		sim_res_t r;

		c.ku = kus[i];
		r = run_sim(&c);
		CHECK(r.released);
		CHECK(!r.fell);
		CHECK(r.max_theta_deg < 12.0);
		CHECK(r.tail_theta_deg < 3.0);
		CHECK(r.max_x_m < 1.5);
		CHECK(r.tail_speed_mps < 0.15);
	}
}

static void sim_kalman_mode_balances_across_plant_gains(void)
{
	const double kus[] = { 0.02, 0.05, 0.1, 0.2 };
	size_t i;

	for (i = 0; i < sizeof(kus) / sizeof(kus[0]); i++) {
		sim_cfg_t c = cfg_default();
		sim_res_t r;

		c.p.est_mode = 1;
		c.ku = kus[i];
		r = run_sim(&c);
		CHECK(!r.fell);
		CHECK(r.tail_theta_deg < 3.0);
		CHECK(r.max_x_m < 1.5);
	}
}

static void sim_tolerates_sensor_noise(void)
{
	sim_cfg_t c = cfg_default();
	sim_res_t r;

	c.accel_noise_g = 0.03; c.gyro_noise_dps = 0.5;
	r = run_sim(&c);
	CHECK(!r.fell);
	CHECK(r.tail_theta_deg < 3.0);
}

static void sim_recovers_from_push(void)
{
	sim_cfg_t c = cfg_default();
	sim_res_t r;

	c.theta0_deg = 0.0; c.push_at_s = 2.0; c.push_rad_s = 0.6;	/* ~34 dps */
	r = run_sim(&c);
	CHECK(!r.fell);
	CHECK(r.tail_theta_deg < 3.0);
}

static void sim_position_hold_stays_stable_and_bounded(void)
{
	sim_cfg_t c = cfg_default();
	sim_res_t off, on;

	c.theta0_deg = 0.0; c.push_at_s = 2.0; c.push_rad_s = 0.6;
	c.seconds = 12.0;
	off = run_sim(&c);
	c.p.x_kp = 0.5f;
	on = run_sim(&c);
	CHECK(!off.fell && !on.fell);
	printf("  drift without hold %.3f m, with hold %.3f m\n", off.final_x_m, on.final_x_m);
	CHECK(fabs(on.final_x_m) <= fabs(off.final_x_m) + 0.02);
	CHECK(fabs(on.final_x_m) < 0.5);
	CHECK(on.tail_theta_deg < 3.0);
}

static void sim_dmp_mode_balances_with_a_good_dmp_angle(void)
{
	sim_cfg_t c = cfg_default();
	sim_res_t r;

	c.p.est_mode = 2;
	c.dmp = 1;
	r = run_sim(&c);
	CHECK(!r.fell);
	CHECK(r.tail_theta_deg < 3.0);
	CHECK(r.dmp_used_cycles > 1000);		/* it really was in the loop */
}

static void sim_dmp_mode_survives_a_frozen_dmp(void)
{
	/* the DMP stops updating (reports 0 deg forever): its disagreement with the always
	 * running filter must hand control back to the filter */
	sim_cfg_t c = cfg_default();
	sim_res_t r;

	c.p.est_mode = 2;
	c.dmp = 2;
	c.theta0_deg = 7.0;
	r = run_sim(&c);
	printf("  frozen DMP: fell=%d max=%.1f deg fallbacks=%u\n", r.fell, r.max_theta_deg, r.dmp_fallbacks);
	CHECK(!r.fell);
	CHECK(r.dmp_fallbacks > 100);
}

static void sim_starts_from_larger_lean(void)
{
	sim_cfg_t c = cfg_default();
	sim_res_t r;

	c.theta0_deg = 12.0;
	r = run_sim(&c);
	CHECK(!r.fell);
	CHECK(r.tail_theta_deg < 3.0);
}

static void sim_speed_loop_stable_up_to_4_deg_per_mps(void)
{
	sim_cfg_t c = cfg_default();
	sim_res_t r;

	c.p.v_kp = 3.9f; c.p.v_ki = 3.9f;
	r = run_sim(&c);
	CHECK(!r.fell);
	CHECK(r.tail_theta_deg < 3.0);
	CHECK(r.max_x_m < 1.0);
}

static void sim_speed_gain_magnitude_limit(void)
{
	sim_cfg_t c = cfg_default();
	sim_res_t r;

	c.p.v_kp = 40.0f; c.p.v_ki = 40.0f;		/* ~20x too high */
	r = run_sim(&c);
	CHECK(r.fell || r.max_theta_deg > 45.0);
}

/* Guards the sign bring-up (docs/04 section 7): wrong encoder polarity turns
 * the speed loop into positive feedback. */
static void sim_wrong_encoder_sign_runs_away_or_falls(void)
{
	sim_cfg_t c = cfg_default();
	sim_res_t r;

	c.p.enc_sign_l = -1; c.p.enc_sign_r = -1;
	c.p.v_kp = 6.0f; c.p.v_ki = 2.0f;
	r = run_sim(&c);
	CHECK(r.fell || r.max_x_m > 5.0 || r.max_theta_deg > 45.0);
}

/* Design rationale (docs/03 section 3.1): low gyro weight without the accel
 * gate lets the car's own acceleration corrupt the angle. */
static void sim_rationale_low_alpha_without_gate_falls(void)
{
	sim_cfg_t c = cfg_default();
	sim_res_t r;

	c.p.est_alpha = 0.98f; c.p.est_gate_g = 0.0f; c.p.est_bias_gain = 0.0f;
	c.p.a_kp = 25.0f;
	r = run_sim(&c);
	CHECK(r.fell || r.max_theta_deg > 45.0);
}

static void sim_imu_unplug_coasts_within_four_cycles(void)
{
	sim_cfg_t c = cfg_default();
	sim_res_t r;

	c.unplug_at_s = 3.0; c.seconds = 4.0;
	r = run_sim(&c);
	CHECK_EQ(r.final_state, BC_ST_FAULT);		/* HIL-06 / S1 */
	CHECK_EQ(r.final_fault, BC_FAULT_IMU_BUS);
	CHECK(r.cycles_to_fault >= 0 && r.cycles_to_fault <= 4);
	CHECK_NEAR(r.motor_after_fault, 0.0, 1e-9);	/* no stale PWM */
}

void suite_sim(void)
{
	printf("suite closed-loop simulation (real bc_core)\n");
	RUN(sim_startup_sequence_and_arming_time);
	RUN(sim_default_params_balance_across_plant_gains);
	RUN(sim_kalman_mode_balances_across_plant_gains);
	RUN(sim_tolerates_sensor_noise);
	RUN(sim_recovers_from_push);
	RUN(sim_dmp_mode_balances_with_a_good_dmp_angle);
	RUN(sim_dmp_mode_survives_a_frozen_dmp);
	RUN(sim_position_hold_stays_stable_and_bounded);
	RUN(sim_starts_from_larger_lean);
	RUN(sim_speed_loop_stable_up_to_4_deg_per_mps);
	RUN(sim_speed_gain_magnitude_limit);
	RUN(sim_wrong_encoder_sign_runs_away_or_falls);
	RUN(sim_rationale_low_alpha_without_gate_falls);
	RUN(sim_imu_unplug_coasts_within_four_cycles);
}
