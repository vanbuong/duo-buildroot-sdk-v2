#include "pid.h"

void pid_init(bc_pid_t *pid, float kp, float ki, float kd,
	      float out_min, float out_max)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->integral = 0.0f;
	pid->prev_err = 0.0f;
	pid->out_min = out_min;
	pid->out_max = out_max;
	pid->i_min = out_min;
	pid->i_max = out_max;
}

void pid_reset(bc_pid_t *pid)
{
	pid->integral = 0.0f;
	pid->prev_err = 0.0f;
}

float pid_update(bc_pid_t *pid, float setpoint, float measurement, float dt)
{
	float err = setpoint - measurement;
	float deriv;
	float out;

	if (dt <= 0.0f)
		dt = 0.001f;

	pid->integral += err * dt;
	if (pid->integral > pid->i_max)
		pid->integral = pid->i_max;
	else if (pid->integral < pid->i_min)
		pid->integral = pid->i_min;

	deriv = (err - pid->prev_err) / dt;
	pid->prev_err = err;

	out = pid->kp * err + pid->ki * pid->integral + pid->kd * deriv;
	if (out > pid->out_max)
		out = pid->out_max;
	else if (out < pid->out_min)
		out = pid->out_min;

	return out;
}
