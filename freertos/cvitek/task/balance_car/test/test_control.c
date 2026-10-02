#include <math.h>
#include "unity_lite.h"
#include "control.h"

static bc_ctrl_in_t base_in(void)
{
	bc_ctrl_in_t in = { 0 };

	in.dt = 0.005f;
	return in;
}

static void ctl_counts_to_mps(void)
{
	bc_params_t p;

	bc_params_default(&p);
	/* 3960 counts/rev, 65 mm wheel -> 19390 counts/m */
	CHECK_NEAR(bc_counts_to_mps(19390, 1.0f, &p), 1.0, 0.001);
	CHECK_NEAR(bc_counts_to_mps(97, 0.005f, &p), 1.0, 0.01);
	CHECK_NEAR(bc_counts_to_mps(-97, 0.005f, &p), -1.0, 0.01);
	CHECK_NEAR(bc_counts_to_mps(5, 0.0f, &p), 0.0, 1e-9);	/* dt guard */
}

static void ctl_slew_limits_step(void)
{
	CHECK_NEAR(bc_slew(0.0f, 1.0f, 0.1f), 0.1, 1e-6);	/* CTL-04 */
	CHECK_NEAR(bc_slew(0.95f, 1.0f, 0.1f), 1.0, 1e-6);
	CHECK_NEAR(bc_slew(0.0f, -1.0f, 0.1f), -0.1, 1e-6);
}

static void ctl_deadband_comp_table(void)
{
	CHECK_NEAR(bc_deadband_comp(0.2f, 6.0f, 90.0f), 0.0, 1e-9);
	CHECK_NEAR(bc_deadband_comp(10.0f, 6.0f, 90.0f), 6.0 + 10.0 * 0.94, 1e-4);
	CHECK_NEAR(bc_deadband_comp(-10.0f, 6.0f, 90.0f), -(6.0 + 10.0 * 0.94), 1e-4);
	CHECK_NEAR(bc_deadband_comp(100.0f, 6.0f, 90.0f), 90.0, 1e-6);
	CHECK_NEAR(bc_deadband_comp(-100.0f, 6.0f, 90.0f), -90.0, 1e-6);
}

static void ctl_upright_still_gives_zero_drive(void)
{
	bc_params_t p;
	bc_ctrl_t c;
	bc_ctrl_in_t in = base_in();
	bc_ctrl_out_t o;

	bc_params_default(&p);
	bc_ctrl_init(&c, &p);
	bc_ctrl_step(&c, &p, &in, &o);
	CHECK_NEAR(o.u, 0.0, 1e-6);
	CHECK_NEAR(o.out_l, 0.0, 1e-6);
	CHECK_NEAR(o.out_r, 0.0, 1e-6);
	CHECK(!o.saturated);
}

static void ctl_angle_pd_matches_formula_and_sign(void)
{
	bc_params_t p;
	bc_ctrl_t c;
	bc_ctrl_in_t in = base_in();
	bc_ctrl_out_t o;

	bc_params_default(&p);
	bc_ctrl_init(&c, &p);
	in.theta = 2.0f;
	in.omega = 5.0f;
	bc_ctrl_step(&c, &p, &in, &o);
	/* u = Kp*(0-2) - Kd*5 = -30 - 4 = -34 : positive angle => negative drive */
	CHECK_NEAR(o.u, -34.0, 1e-3);
	CHECK(o.out_l < 0 && o.out_r < 0);
}

static void ctl_trim_shifts_balance_point(void)
{
	bc_params_t p;
	bc_ctrl_t c;
	bc_ctrl_in_t in = base_in();
	bc_ctrl_out_t o;

	bc_params_default(&p);
	p.trim_deg = 2.0f;
	bc_ctrl_init(&c, &p);
	in.theta = 2.0f;
	bc_ctrl_step(&c, &p, &in, &o);
	CHECK_NEAR(o.u, 0.0, 1e-4);
}

static void ctl_saturation_flag_and_limit(void)
{
	bc_params_t p;
	bc_ctrl_t c;
	bc_ctrl_in_t in = base_in();
	bc_ctrl_out_t o;

	bc_params_default(&p);
	bc_ctrl_init(&c, &p);
	in.theta = -20.0f;
	bc_ctrl_step(&c, &p, &in, &o);
	CHECK(o.saturated);
	CHECK_NEAR(o.u, 90.0, 1e-4);
	CHECK(o.out_l <= p.motor_max_pct + 1e-3);
}

static void ctl_speed_loop_leans_back_when_moving_forward(void)
{
	/* docs/04 section 7: moving forward => lean back (positive angle cmd) */
	bc_params_t p;
	bc_ctrl_t c;
	bc_ctrl_in_t in = base_in();
	bc_ctrl_out_t o;
	int i;

	bc_params_default(&p);
	bc_ctrl_init(&c, &p);
	for (i = 0; i < 40; i++) {
		in.enc_l += 100;		/* ~1 m/s forward */
		in.enc_r += 100;
		bc_ctrl_step(&c, &p, &in, &o);
	}
	CHECK(o.v_f > 0.5f);
	CHECK(o.theta_cmd > 1.0f);			/* CTL-02 */
	CHECK(o.theta_cmd <= p.v_max_deg + 1e-4);
}

static void ctl_encoder_sign_flips_speed_sign(void)
{
	bc_params_t p;
	bc_ctrl_t c;
	bc_ctrl_in_t in = base_in();
	bc_ctrl_out_t o;
	int i;

	bc_params_default(&p);
	p.enc_sign_l = p.enc_sign_r = -1;
	bc_ctrl_init(&c, &p);
	for (i = 0; i < 40; i++) {
		in.enc_l += 100;
		in.enc_r += 100;
		bc_ctrl_step(&c, &p, &in, &o);
	}
	CHECK(o.v_f < -0.5f);
	CHECK(o.theta_cmd < -1.0f);
}

static void ctl_target_is_slew_limited_and_clamped(void)
{
	bc_params_t p;
	bc_ctrl_t c;
	bc_ctrl_in_t in = base_in();
	bc_ctrl_out_t o;
	int i;

	bc_params_default(&p);
	bc_ctrl_init(&c, &p);
	in.v_target = 5.0f;				/* way above v_max */
	bc_ctrl_step(&c, &p, &in, &o);
	CHECK_NEAR(c.v_t, p.acc_max * 0.005, 1e-5);
	for (i = 0; i < 400; i++)
		bc_ctrl_step(&c, &p, &in, &o);
	CHECK_NEAR(c.v_t, p.v_max, 1e-5);
}

static void ctl_turn_makes_differential_drive(void)
{
	bc_params_t p;
	bc_ctrl_t c;
	bc_ctrl_in_t in = base_in();
	bc_ctrl_out_t o;
	int i;

	bc_params_default(&p);
	bc_ctrl_init(&c, &p);
	in.w_target = 1.0f;
	for (i = 0; i < 100; i++)
		bc_ctrl_step(&c, &p, &in, &o);
	CHECK(o.turn > 1.0f);
	CHECK(o.out_l > o.out_r);			/* CTL-05 */
}

static void ctl_mixer_keeps_balance_drive_over_turn(void)
{
	bc_params_t p;
	bc_ctrl_t c;
	bc_ctrl_in_t in = base_in();
	bc_ctrl_out_t o;
	int i;

	bc_params_default(&p);
	bc_ctrl_init(&c, &p);
	in.theta = -20.0f;				/* u saturates at +90 */
	in.w_target = 2.0f;
	for (i = 0; i < 400; i++)
		bc_ctrl_step(&c, &p, &in, &o);
	CHECK_NEAR(o.turn, 0.0, 1e-4);			/* no room left for turning */
	CHECK_NEAR(o.out_l, o.out_r, 1e-3);
}

static void ctl_reset_clears_state(void)
{
	bc_params_t p;
	bc_ctrl_t c;
	bc_ctrl_in_t in = base_in();
	bc_ctrl_out_t o;
	int i;

	bc_params_default(&p);
	bc_ctrl_init(&c, &p);
	in.v_target = 0.4f;
	for (i = 0; i < 200; i++)
		bc_ctrl_step(&c, &p, &in, &o);
	bc_ctrl_reset(&c);
	CHECK_NEAR(c.v_t, 0.0, 1e-9);
	CHECK_NEAR(c.v_f, 0.0, 1e-9);
	CHECK_NEAR(c.pv.integral, 0.0, 1e-9);
	CHECK_NEAR(c.pa.integral, 0.0, 1e-9);
}

void suite_control(void)
{
	printf("suite control\n");
	RUN(ctl_counts_to_mps);
	RUN(ctl_slew_limits_step);
	RUN(ctl_deadband_comp_table);
	RUN(ctl_upright_still_gives_zero_drive);
	RUN(ctl_angle_pd_matches_formula_and_sign);
	RUN(ctl_trim_shifts_balance_point);
	RUN(ctl_saturation_flag_and_limit);
	RUN(ctl_speed_loop_leans_back_when_moving_forward);
	RUN(ctl_encoder_sign_flips_speed_sign);
	RUN(ctl_target_is_slew_limited_and_clamped);
	RUN(ctl_turn_makes_differential_drive);
	RUN(ctl_mixer_keeps_balance_drive_over_turn);
	RUN(ctl_reset_clears_state);
}
