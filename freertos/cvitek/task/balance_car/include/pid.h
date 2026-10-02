#ifndef BALANCE_PID_H
#define BALANCE_PID_H

typedef struct {
	float kp;
	float ki;
	float kd;
	float integral;
	float prev_err;
	float out_min;
	float out_max;
	float i_min;
	float i_max;
} bc_pid_t;

void pid_init(bc_pid_t *pid, float kp, float ki, float kd,
	      float out_min, float out_max);
void pid_reset(bc_pid_t *pid);
float pid_update(bc_pid_t *pid, float setpoint, float measurement, float dt);

/* Limit the integral TERM (ki * integral) to +-term_limit, independent of the
 * output limits. No effect while ki == 0. Call again after changing ki. */
void pid_set_i_limit(bc_pid_t *pid, float term_limit);

/*
 * PID with the derivative taken from an externally supplied measurement rate
 * (e.g. the gyro): out = kp*err + ki*I - kd*rate. No derivative kick on
 * setpoint changes. If freeze != 0 the integrator holds (anti-windup while
 * the output is saturated or the controller is inactive).
 */
float pid_update_rate(bc_pid_t *pid, float setpoint, float measurement,
		      float rate, float dt, int freeze);

#endif /* BALANCE_PID_H */
