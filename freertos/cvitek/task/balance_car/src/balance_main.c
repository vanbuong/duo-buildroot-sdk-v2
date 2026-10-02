/*
 * Balance-car application on the C906L (FreeRTOS).
 *
 * bc_ctrl     prio idle+6  200 Hz, vTaskDelayUntil(1 tick): IMU -> bc_core -> TB6612
 * BALANCE     prio idle+4  mailbox commands from Linux (balance_comm.c, created by comm_main.c)
 * bc_console  prio idle+2  drains the event ring to the UART (printf never runs in bc_ctrl)
 *
 * All algorithmic decisions live in bc_core (host-tested). docs/balance_car/01,04.
 */
#include <stdio.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "printf.h"
#include "delay.h"
#include "arch_time.h"
#include "arch_cpu.h"

#include "balance_car.h"
#include "balance_rt.h"
#include "board_pins.h"
#include "tb6612.h"
#include "encoder.h"
#include "enc_irq.h"
#include "mpu60x0.h"
#include "bc_core.h"
#include "bc_cmd.h"
#include "bc_proto.h"

#ifndef BC_BUILD_ID
#define BC_BUILD_ID	__DATE__
#endif

#define PRIO_CTRL	(tskIDLE_PRIORITY + 6)
#define PRIO_CONSOLE	(tskIDLE_PRIORITY + 2)
#define TELEM_DIV	4			/* 200 Hz -> 50 Hz records   */
#define HB_DIV		20			/* RTOS heartbeat at 10 Hz   */

static tb6612_t g_motors;
static encoder_t g_enc_l, g_enc_r;
static mpu60x0_t g_imu;
static bc_core_t g_core;
static bc_params_t g_params;
static bc_cmd_t g_cmd;
static int g_motors_on;

uint32_t bc_now_us(void)
{
	return (uint32_t)(GetSysTime() / (configSYS_CLOCK_HZ / 1000000UL));
}

uint32_t bc_now_ms(void)
{
	return (uint32_t)(GetSysTime() / (configSYS_CLOCK_HZ / 1000UL));
}

static void load_default_params(bc_params_t *p)
{
	bc_params_default(p);
	p->imu_sign = BC_BOARD_IMU_SIGN;
	p->motor_sign_l = BC_BOARD_MOTOR_SIGN_L;
	p->motor_sign_r = BC_BOARD_MOTOR_SIGN_R;
	p->enc_sign_l = BC_BOARD_ENC_SIGN_L;
	p->enc_sign_r = BC_BOARD_ENC_SIGN_R;
	p->enc_cpr = (float)BC_ENC_COUNTS_PER_REV;
}

static int pct(float x)
{
	return (int)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

/* Only BALANCING drives the motors; every other state forces coast + STBY=0. */
static void apply_outputs(const bc_core_out_t *o, unsigned cycle)
{
	if (o->motor_enable) {
		if (!g_motors_on) {
			tb6612_enable(&g_motors, 1);
			g_motors_on = 1;
		}
		tb6612_set_speed(&g_motors, TB6612_LEFT, pct(o->out_l));
		tb6612_set_speed(&g_motors, TB6612_RIGHT, pct(o->out_r));
	} else if (g_motors_on || (cycle % HB_DIV) == 0) {
		tb6612_coast(&g_motors);
		tb6612_enable(&g_motors, 0);
		g_motors_on = 0;
	}
}

static int16_t s16(float x)
{
	return (int16_t)(x > 32767.0f ? 32767.0f : (x < -32768.0f ? -32768.0f : x));
}

static void publish_telemetry(const bc_core_out_t *o, const bc_core_in_t *in,
			      uint32_t period_us, uint32_t exec_us,
			      const bc_cmd_t *cmd, int zeroed, int hb_lost,
			      uint32_t imu_err)
{
	volatile bc_shm_t *shm = BC_SHM();
	bc_telem_t t = { 0 };

	t.t_us = bc_now_us();
	t.pitch_cdeg = s16(o->theta * 100.0f);
	t.pitch_acc_cdeg = s16(o->theta_acc * 100.0f);
	t.gyro_y_cdps = s16(o->omega * 100.0f);
	t.gyro_z_cdps = s16(in->gz * 100.0f);
	t.accel_norm_mg = s16(o->norm * 1000.0f);
	t.speed_l_mmps = s16(o->ctl.v_l * 1000.0f);
	t.speed_r_mmps = s16(o->ctl.v_r * 1000.0f);
	t.angle_set_cdeg = s16(o->ctl.theta_cmd * 100.0f);
	t.motor_l_pm = s16(o->out_l * 10.0f);
	t.motor_r_pm = s16(o->out_r * 10.0f);
	t.target_speed_mmps = (int16_t)(cmd->speed_mmps);
	t.target_turn_mradps = (int16_t)(cmd->turn_mradps);
	t.enc_l = in->enc_l;
	t.enc_r = in->enc_r;
	t.exec_us = (uint16_t)(exec_us > 65535 ? 65535 : exec_us);
	t.period_us = (uint16_t)(period_us > 65535 ? 65535 : period_us);
	t.state = (uint8_t)o->state;
	t.fault = (uint8_t)o->fault;
	t.imu_err = (uint8_t)(imu_err > 255 ? 255 : imu_err);
	t.flags = (uint8_t)((o->gated ? 1 : 0) | (zeroed ? 2 : 0) | (hb_lost ? 4 : 0) |
			    (o->ctl.saturated ? 8 : 0));
	bc_telem_push(shm, &t);
}

/* Average the estimator over ~1 s while IDLE (robot held at its balance point). */
static int calibrate_trim(float *trim_out, TickType_t *last)
{
	float sum = 0.0f;
	int i, n = 0;

	for (i = 0; i < BC_CTRL_HZ; i++) {
		mpu60x0_scaled_t sc;
		bc_core_in_t in = { 0 };
		bc_core_out_t out = { 0 };

		vTaskDelayUntil(last, 1);
		if (mpu60x0_read(&g_imu))
			return -1;
		mpu60x0_scale(&g_imu, &sc);
		in.imu_ok = 1;
		in.raw = g_imu.raw;
		in.ax = sc.ax; in.ay = sc.ay; in.az = sc.az;
		in.gy = sc.gy; in.gz = sc.gz;
		in.raw_gx = g_imu.gx; in.raw_gy = g_imu.gy; in.raw_gz = g_imu.gz;
		in.dt = 1.0f / (float)BC_CTRL_HZ;
		bc_core_step(&g_core, &in, &out);
		sum += out.theta;
		n++;
	}
	*trim_out = sum / (float)n;
	return (fabsf(*trim_out) <= 15.0f) ? 0 : -2;
}

static void balance_ctrl_task(void *arg)
{
	volatile bc_shm_t *shm = BC_SHM();
	TickType_t last;
	uint32_t boot_events[2] = { 0, 0 };
	unsigned boot_idx = 0;
	uint32_t t_prev, rtos_hb = 0, last_shm_hb = 0, last_mbox_hb_ms = 0;
	uint32_t hb_fake = 0, miss_window = 0, miss_count = 0;
	unsigned cycle = 0;
	bc_state_id_t prev_state = BC_ST_BOOT;
	bc_fault_t prev_fault = BC_FAULT_NONE;
	bc_core_out_t out = { 0 };
	int zeroed = 0, hb_lost = 0;
	float v_t = 0.0f, w_t = 0.0f;

	(void)arg;
	load_default_params(&g_params);
	bc_core_init(&g_core, &g_params);
	bc_cmd_init(&g_cmd, bc_now_ms());
	bc_param_echo_write(shm, &g_params, 0, 0);

	if (mpu60x0_init(&g_imu, BC_MPU_I2C_ID, BC_MPU_ADDR) != 0) {
		boot_events[0] = BC_EV_IMU_INIT_FAIL;
		bc_event_push(shm, bc_now_us(), BC_EVT_LOG_CALIB, BC_CALIB_GYRO, -1);
	} else {
		int rc = mpu60x0_calibrate(&g_imu, 200);

		boot_events[0] = BC_EV_CALIB_REQ;
		boot_events[1] = rc == 0 ? BC_EV_CALIB_OK : BC_EV_CALIB_FAIL;
		bc_event_push(shm, bc_now_us(), BC_EVT_LOG_CALIB, BC_CALIB_GYRO, rc);
	}

	last = xTaskGetTickCount();
	t_prev = bc_now_us();
	for (;;) {
		bc_core_in_t in = { 0 };
		mpu60x0_scaled_t sc = { 0 };
		uint32_t t0, period_us, exec_us, ev, mb_ms, mb_hb, tgt_ms, imu_err;
		int32_t spd, trn;
		float dt;
		int rc;
		bc_params_t np;
		uint32_t pseq;

		vTaskDelayUntil(&last, 1);
		t0 = bc_now_us();
		period_us = t0 - t_prev;
		t_prev = t0;
		dt = (float)period_us * 1e-6f;
		if (dt < 0.002f) dt = 0.002f;
		if (dt > 0.010f) dt = 0.010f;

		/* deadline accounting: 3 misses within one second -> timing fault */
		if (period_us > 7500u && cycle > 10)
			miss_count++;
		if (++miss_window >= BC_CTRL_HZ) {
			miss_window = 0;
			if (miss_count)
				shm->status.deadline_miss += miss_count;
			miss_count = 0;
		}

		/* --- sensors --- */
#if !BC_ENC_USE_IRQ
		encoder_poll(&g_enc_l);
		encoder_poll(&g_enc_r);
#endif
		rc = mpu60x0_read(&g_imu);
		if (rc == 0)
			mpu60x0_scale(&g_imu, &sc);

		/* --- commands from Linux --- */
		bc_rt_get_cmd(&spd, &trn, &tgt_ms, &mb_hb, &mb_ms);
		g_cmd.speed_mmps = spd;
		g_cmd.turn_mradps = trn;
		g_cmd.target_ms = tgt_ms;
		if ((cycle % 10) == 0) {		/* shm heartbeat, 20 Hz */
			uint32_t shb;

			BC_SHM_INV(&shm->lnx, sizeof(shm->lnx));
			shb = shm->lnx.linux_heartbeat;
			if (shb != last_shm_hb) {
				last_shm_hb = shb;
				bc_cmd_heartbeat(&g_cmd, ++hb_fake, bc_now_ms());
			}
		}
		if (mb_ms != last_mbox_hb_ms) {
			last_mbox_hb_ms = mb_ms;
			bc_cmd_heartbeat(&g_cmd, ++hb_fake, bc_now_ms());
		}
		bc_cmd_effective(&g_cmd, &g_params, bc_now_ms(), &v_t, &w_t, &zeroed, &hb_lost);

		/* --- events and parameter updates --- */
		ev = bc_rt_take_events();
		if (boot_idx < 2 && boot_events[boot_idx])
			ev |= boot_events[boot_idx++];
		if (hb_lost)
			ev |= BC_EV_DISARM;
		if (bc_rt_take_params(&np, &pseq)) {
			bc_core_set_params(&g_core, &np);
			g_params = np;
			bc_param_echo_write(shm, &np, 0, pseq);
		}

		/* --- one control cycle --- */
		in.imu_ok = (rc == 0);
		in.raw = g_imu.raw;
		in.ax = sc.ax; in.ay = sc.ay; in.az = sc.az;
		in.gy = sc.gy; in.gz = sc.gz;
		in.raw_gx = g_imu.gx; in.raw_gy = g_imu.gy; in.raw_gz = g_imu.gz;
		in.enc_l = encoder_get_count(&g_enc_l);
		in.enc_r = encoder_get_count(&g_enc_r);
		in.v_target = v_t;
		in.w_target = w_t;
		in.dt = dt;
		in.events = ev;
		in.timing_fault = (miss_count >= 3);
		bc_core_step(&g_core, &in, &out);
		apply_outputs(&out, cycle);

		/* --- runtime calibration requests (only while disarmed) --- */
		{
			uint32_t cal = bc_rt_take_calib();

			if (cal && out.state == BC_ST_IDLE) {
				int r = -3;
				bc_core_in_t ci = { 0 };
				bc_core_out_t co = { 0 };

				ci.imu_ok = 1; ci.dt = dt; ci.events = BC_EV_CALIB_REQ;
				ci.ax = sc.ax; ci.ay = sc.ay; ci.az = sc.az;
				bc_core_step(&g_core, &ci, &co);
				if (cal == BC_CALIB_GYRO) {
					r = mpu60x0_calibrate(&g_imu, 200);
				} else if (cal == BC_CALIB_TRIM) {
					float trim;

					r = calibrate_trim(&trim, &last);
					if (r == 0) {
						bc_params_t np2 = g_params;

						np2.trim_deg = trim;
						bc_core_set_params(&g_core, &np2);
						g_params = np2;
						bc_param_echo_write(shm, &np2, 0, 0);
					}
				}
				ci.events = (r == 0) ? BC_EV_CALIB_OK : BC_EV_CALIB_FAIL;
				bc_core_step(&g_core, &ci, &co);
				bc_event_push(shm, bc_now_us(), BC_EVT_LOG_CALIB, (uint16_t)cal, r);
				bc_rt_send_evt(BC_EVT_CALIB_DONE, (uint32_t)r);
				/* calibration may have blocked for ~1 s: restart the cadence
				 * instead of letting vTaskDelayUntil run ~200 catch-up cycles */
				last = xTaskGetTickCount();
				t_prev = bc_now_us();
			}
		}

		/* --- state change notifications --- */
		if (out.state != prev_state || out.fault != prev_fault) {
			prev_state = out.state;
			prev_fault = out.fault;
			bc_event_push(shm, bc_now_us(), BC_EVT_LOG_STATE, (uint16_t)out.state,
				      (int32_t)out.fault);
			bc_rt_send_evt(BC_EVT_STATE, bc_pack_state(out.state, out.fault, 0));
		}

		/* --- status, heartbeat, telemetry --- */
		imu_err = g_core.health.total_errors;
		exec_us = bc_now_us() - t0;
		shm->status.state = (uint32_t)out.state;
		shm->status.fault = (uint32_t)out.fault;
		shm->status.imu_err = imu_err;
		shm->status.cycles = cycle;
		if ((cycle % HB_DIV) == 0) {
			shm->status.rtos_heartbeat = ++rtos_hb;
			shm->status.stack_min_words[0] = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
			BC_SHM_CLEAN(&shm->status, sizeof(shm->status));
		}
		if ((cycle % TELEM_DIV) == 0)
			publish_telemetry(&out, &in, period_us, exec_us, &g_cmd, zeroed,
					  hb_lost, imu_err);
		cycle++;
	}
}

/* Low-priority console: prints events and a 1 Hz status line. */
static void bc_console_task(void *arg)
{
	volatile bc_shm_t *shm = BC_SHM();
	uint32_t next_ev = 0;
	unsigned tick = 0;

	(void)arg;
	for (;;) {
		bc_event_t e;
		int rc;

		vTaskDelay(pdMS_TO_TICKS(100));
		while ((rc = bc_event_read(shm, next_ev, &e)) <= 0) {
			if (rc == -1) {		/* overwritten: skip ahead */
				next_ev = shm->status.event_head - BC_EVENT_SLOTS + 1;
				continue;
			}
			if (rc == 0) {
				if (e.type == BC_EVT_LOG_STATE)
					printf("[balance] state=%s fault=%s\n",
					       bc_state_name((bc_state_id_t)e.arg),
					       bc_fault_name((bc_fault_t)e.val));
				else
					printf("[balance] ev type=%u arg=%u val=%ld\n",
					       (unsigned)e.type, (unsigned)e.arg, (long)e.val);
			}
			next_ev++;
		}
		if ((++tick % 20) == 0) {
			bc_telem_t t;
			uint32_t head = shm->status.telem_head;

			if (head && bc_telem_read(shm, head - 1, &t) == 0)
				printf("[balance] %s pitch=%d.%02d speed=%d/%dmm/s motor=%d/%d enc=%ld/%ld "
				       "T=%uus exec=%uus miss=%lu\n",
				       bc_state_name((bc_state_id_t)t.state),
				       t.pitch_cdeg / 100, (t.pitch_cdeg < 0 ? -t.pitch_cdeg : t.pitch_cdeg) % 100,
				       t.speed_l_mmps, t.speed_r_mmps, t.motor_l_pm / 10, t.motor_r_pm / 10,
				       (long)t.enc_l, (long)t.enc_r, t.period_us, t.exec_us,
				       (unsigned long)shm->status.deadline_miss);
		}
	}
}

void balance_car_start(void)
{
	volatile bc_shm_t *shm = BC_SHM();
	uint32_t boot = 1;

	printf("[balance] starting DuoS balance-car firmware (IMU I2C%d @0x%02x)\n",
	       BC_MPU_I2C_ID, BC_MPU_ADDR);

	/* shared window first, so Linux can see us even if the IMU is missing */
	BC_SHM_INV(&shm->hdr, sizeof(shm->hdr));
	if (shm->hdr.magic == BC_SHM_MAGIC)
		boot = shm->hdr.boot_count + 1;	/* RTOS reload without a reboot */
	bc_shm_init(shm, boot, BC_BUILD_ID);

	board_pins_init();
	tb6612_init(&g_motors);			/* STBY low: motors off */
	encoder_init(&g_enc_l, BC_PIN_ENC_L_A, BC_PIN_ENC_L_B);
	encoder_init(&g_enc_r, BC_PIN_ENC_R_A, BC_PIN_ENC_R_B);
#if BC_ENC_USE_IRQ
	if (enc_irq_start(&g_enc_l, &g_enc_r))
		printf("[balance] WARNING: encoder IRQ unavailable, speed loop blind\n");
#endif

	xTaskCreate(balance_ctrl_task, "bc_ctrl", configMINIMAL_STACK_SIZE * 4, NULL,
		    PRIO_CTRL, NULL);
	xTaskCreate(bc_console_task, "bc_console", configMINIMAL_STACK_SIZE * 3, NULL,
		    PRIO_CONSOLE, NULL);
}
