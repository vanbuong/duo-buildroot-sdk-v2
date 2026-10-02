/*
 * Run-time parameter set for the balance controller (single source of truth).
 * All fields are 32-bit so the struct has the same layout on the C906L
 * (RV64) and the Linux side (aarch64/python struct): see docs/balance_car/01.
 *
 * Sign convention (docs/balance_car/04 section 7): with the sign macros set
 * correctly, positive angle => negative motor output => wheels move toward
 * the lean. Gains are magnitudes; the speed loop applies its own sign.
 */
#ifndef BC_PARAMS_H
#define BC_PARAMS_H

#include <stdint.h>
#include <stddef.h>

#define BC_T_F float
#define BC_T_I int32_t
#define BC_T_S int32_t

/* X(name, type, default, min, max).  type: F float, I int, S sign (+1/-1). */
#define BC_PARAM_LIST(X) \
	X(est_alpha,      F, 0.998f, 0.90f,  0.9999f) /* complementary gyro weight */ \
	X(est_gate_g,     F, 0.10f,  0.0f,   1.0f)    /* accel norm gate, 0 = off  */ \
	X(est_bias_gain,  F, 0.02f,  0.0f,   1.0f)    /* online gyro bias, 1/s     */ \
	X(a_kp,           F, 15.0f,  3.5f,   60.0f)   /* % per deg                 */ \
	X(a_kd,           F, 0.8f,   0.0f,   5.0f)    /* % per (deg/s)             */ \
	X(a_ki,           F, 0.0f,   0.0f,   5.0f)    /* % per (deg*s)             */ \
	X(trim_deg,       F, 0.0f,  -15.0f,  15.0f)   /* mechanical balance point  */ \
	X(a_out_max,      F, 90.0f,  20.0f,  100.0f)  /* angle loop output limit % */ \
	X(v_kp,           F, 1.94f,  0.0f,   8.0f)    /* deg per (m/s), magnitude  */ \
	X(v_ki,           F, 1.94f,  0.0f,   8.0f)    /* deg per m                 */ \
	X(v_max_deg,      F, 8.0f,   0.0f,   20.0f)   /* speed loop lean limit     */ \
	X(v_filter,       F, 0.3f,   0.01f,  1.0f)    /* wheel speed low-pass      */ \
	X(t_kp,           F, 5.0f,   0.0f,   30.0f)   /* % per (rad/s) yaw rate    */ \
	X(t_ki,           F, 5.0f,   0.0f,   30.0f)   \
	X(t_max,          F, 20.0f,  0.0f,   50.0f)   /* turn authority %          */ \
	X(t_speed_scale,  F, 0.5f,   0.05f,  5.0f)    /* turn shrinks above this m/s */ \
	X(deadband_pct,   F, 6.0f,   0.0f,   20.0f)   /* motor dead zone %         */ \
	X(motor_max_pct,  F, 90.0f,  20.0f,  100.0f)  \
	X(acc_max,        F, 0.5f,   0.05f,  5.0f)    /* m/s^2 target slew         */ \
	X(alpha_max,      F, 3.0f,   0.1f,   20.0f)   /* rad/s^2 yaw target slew   */ \
	X(v_max,          F, 0.5f,   0.0f,   2.0f)    /* m/s                       */ \
	X(w_max,          F, 2.0f,   0.0f,   6.0f)    /* rad/s                     */ \
	X(fall_deg,       F, 45.0f,  20.0f,  80.0f)   \
	X(lift_speed_mps, F, 1.5f,   0.3f,   5.0f)    \
	X(sat_ms,         F, 300.0f, 50.0f,  5000.0f) \
	X(cmd_timeout_ms, F, 500.0f, 100.0f, 10000.0f) /* heartbeat/target timeout */ \
	X(hb_disarm_ms,   F, 0.0f,   0.0f,   600000.0f) /* 0 = never disarm        */ \
	X(wheel_diam_m,   F, 0.065f, 0.02f,  0.3f)    \
	X(track_m,        F, 0.17f,  0.05f,  0.6f)    \
	X(enc_cpr,        F, 3960.0f,100.0f, 100000.0f) /* counts per wheel rev    */ \
	X(imu_sign,       S, 1,     -1,      1)       \
	X(motor_sign_l,   S, 1,     -1,      1)       \
	X(motor_sign_r,   S, 1,     -1,      1)       \
	X(enc_sign_l,     S, 1,     -1,      1)       \
	X(enc_sign_r,     S, 1,     -1,      1)

typedef struct {
#define BC_X_FIELD(n, t, d, lo, hi) BC_T_##t n;
	BC_PARAM_LIST(BC_X_FIELD)
#undef BC_X_FIELD
} bc_params_t;

typedef struct {
	const char *name;
	uint16_t offset;
	char type;		/* 'F', 'I', 'S' */
	float min, max;
} bc_param_desc_t;

extern const bc_param_desc_t bc_param_table[];
extern const unsigned bc_param_count;

void bc_params_default(bc_params_t *p);
/* Returns 0 if every field is finite and in range, else 1 + index of the
 * first bad field. */
int bc_params_validate(const bc_params_t *p);
int bc_param_index(const char *name);			/* -1 if unknown */
double bc_param_get(const bc_params_t *p, unsigned idx);
/* 0 on success, -1 unknown index, -2 out of range / not finite. */
int bc_param_set(bc_params_t *p, unsigned idx, double value);

uint32_t bc_crc32(const void *data, size_t len);

#endif /* BC_PARAMS_H */
