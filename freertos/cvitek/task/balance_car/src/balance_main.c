#include <stdio.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "printf.h"
#include "delay.h"
#include "balance_car.h"
#include "board_pins.h"
#include "tb6612.h"
#include "encoder.h"
#include "mpu60x0.h"
#include "pid.h"

/* Tunable starting gains — retune on the real chassis. */
#define ANGLE_KP	25.0f
#define ANGLE_KI	0.0f
#define ANGLE_KD	0.8f

#define SPEED_KP	0.05f
#define SPEED_KI	0.01f
#define SPEED_KD	0.0f

#define FALL_ANGLE_DEG	45.0f
#define MAX_MOTOR_PCT	90

static tb6612_t g_motors;
static encoder_t g_enc_l;
static encoder_t g_enc_r;
static mpu60x0_t g_imu;
static bc_pid_t g_pid_angle;
static bc_pid_t g_pid_speed;

static volatile float g_target_speed;
static volatile float g_target_turn;
static volatile int g_armed;

/*
 * Fast IO loop on the little core: poll encoders continuously and run the
 * balance PID every BC_CTRL_HZ. Hardware PWM frees the CPU from bit-banging.
 */
static void balance_ctrl_task(void *arg)
{
	TickType_t last_ctrl = xTaskGetTickCount();
	const TickType_t ctrl_period = pdMS_TO_TICKS(1000 / BC_CTRL_HZ);
	float dt = 1.0f / (float)BC_CTRL_HZ;
	int32_t prev_l = 0, prev_r = 0;
	unsigned log_div = 0;

	(void)arg;
	printf("[balance] control+encoder loop @ %d Hz (enc polled continuously)\n",
	       BC_CTRL_HZ);

	for (;;) {
		TickType_t now;

		encoder_poll(&g_enc_l);
		encoder_poll(&g_enc_r);

		now = xTaskGetTickCount();
		if ((now - last_ctrl) < ctrl_period) {
			/* Yield briefly so mailbox / idle can run. */
			udelay(40);
			continue;
		}
		last_ctrl = now;

		{
			float angle;
			float speed;
			float angle_set;
			float motor;
			int left, right;
			int32_t cl, cr;

			if (mpu60x0_read(&g_imu) != 0)
				continue;
			mpu60x0_update_angle(&g_imu, dt);
			angle = g_imu.angle_deg;

			cl = encoder_get_count(&g_enc_l);
			cr = encoder_get_count(&g_enc_r);
			speed = (float)((cl - prev_l) + (cr - prev_r)) * 0.5f / dt;
			prev_l = cl;
			prev_r = cr;

			if (!g_armed) {
				tb6612_coast(&g_motors);
				pid_reset(&g_pid_angle);
				pid_reset(&g_pid_speed);
				continue;
			}

			if (fabsf(angle) > FALL_ANGLE_DEG) {
				printf("[balance] fall detect angle=%.1f — disarm\n",
				       angle);
				g_armed = 0;
				tb6612_coast(&g_motors);
				tb6612_enable(&g_motors, 0);
				continue;
			}

			angle_set = pid_update(&g_pid_speed, g_target_speed,
					       speed, dt);
			motor = pid_update(&g_pid_angle, angle_set, angle, dt);

			left = (int)(motor + g_target_turn);
			right = (int)(motor - g_target_turn);
			if (left > MAX_MOTOR_PCT)
				left = MAX_MOTOR_PCT;
			if (left < -MAX_MOTOR_PCT)
				left = -MAX_MOTOR_PCT;
			if (right > MAX_MOTOR_PCT)
				right = MAX_MOTOR_PCT;
			if (right < -MAX_MOTOR_PCT)
				right = -MAX_MOTOR_PCT;

			tb6612_set_speed(&g_motors, TB6612_LEFT, left);
			tb6612_set_speed(&g_motors, TB6612_RIGHT, right);

			if ((log_div++ % (BC_CTRL_HZ * 2)) == 0) {
				printf("[balance] ang=%.2f tgt=%.2f spd=%.0f L=%d R=%d enc=%ld/%ld\n",
				       angle, angle_set, speed, left, right,
				       (long)cl, (long)cr);
			}
		}
	}
}

void balance_car_start(void)
{
	printf("[balance] starting DuoS balance-car firmware\n");
	printf("[balance] MPU I2C%d @0x%02x, TB6612 + 2x JGB37-520 encoders\n",
	       BC_MPU_I2C_ID, BC_MPU_ADDR);

	board_pins_init();
	tb6612_init(&g_motors);

	encoder_init(&g_enc_l, BC_PIN_ENC_L_A, BC_PIN_ENC_L_B);
	encoder_init(&g_enc_r, BC_PIN_ENC_R_A, BC_PIN_ENC_R_B);

	if (mpu60x0_init(&g_imu, BC_MPU_I2C_ID, BC_MPU_ADDR) != 0) {
		printf("[balance] IMU init failed — motors stay disabled\n");
		g_armed = 0;
	} else {
		mpu60x0_calibrate_gyro(&g_imu, 200);
		tb6612_enable(&g_motors, 1);
		g_armed = 1;
		printf("[balance] armed — keep robot upright\n");
	}

	pid_init(&g_pid_angle, ANGLE_KP, ANGLE_KI, ANGLE_KD,
		 (float)(-MAX_MOTOR_PCT), (float)MAX_MOTOR_PCT);
	pid_init(&g_pid_speed, SPEED_KP, SPEED_KI, SPEED_KD, -15.0f, 15.0f);

	g_target_speed = 0.0f;
	g_target_turn = 0.0f;

	xTaskCreate(balance_ctrl_task, "balance", configMINIMAL_STACK_SIZE * 4,
		    NULL, tskIDLE_PRIORITY + 5, NULL);
}
