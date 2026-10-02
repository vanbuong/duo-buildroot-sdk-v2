/*
 * Linux <-> RTOS command handling for the balance firmware.
 * Messages arrive as 8-byte cmdqu_t from the mailbox ISR (ip_id = IP_BALANCE,
 * routed by comm_main.c) and are replied to by sending BC_EVT_* messages
 * through the existing CMDQU task. docs/balance_car/01 section 6.
 */
#include <stdio.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "printf.h"
#include "rtos_cmdqu.h"
#include "comm_def.h"
#include "comm.h"
#include "arch_helpers.h"

#include "balance_rt.h"
#include "bc_proto.h"
#include "bc_state.h"

static volatile uint32_t g_events;		/* BC_EV_* bits */
static volatile uint32_t g_calib_req;
static int32_t g_speed_mmps, g_turn_mradps;
static uint32_t g_target_ms, g_hb_counter, g_hb_ms;
static bc_params_t g_pending_params;
static uint32_t g_pending_seq;
static volatile int g_have_params;

uint32_t bc_rt_take_events(void)
{
	return __atomic_exchange_n(&g_events, 0u, __ATOMIC_ACQ_REL);
}

uint32_t bc_rt_take_calib(void)
{
	return __atomic_exchange_n(&g_calib_req, 0u, __ATOMIC_ACQ_REL);
}

void bc_rt_get_cmd(int32_t *speed_mmps, int32_t *turn_mradps, uint32_t *target_ms,
		   uint32_t *hb_counter, uint32_t *hb_ms)
{
	taskENTER_CRITICAL();
	*speed_mmps = g_speed_mmps;
	*turn_mradps = g_turn_mradps;
	*target_ms = g_target_ms;
	*hb_counter = g_hb_counter;
	*hb_ms = g_hb_ms;
	taskEXIT_CRITICAL();
}

int bc_rt_take_params(bc_params_t *p, uint32_t *seq)
{
	int have;

	taskENTER_CRITICAL();
	have = g_have_params;
	if (have) {
		*p = g_pending_params;
		*seq = g_pending_seq;
		g_have_params = 0;
	}
	taskEXIT_CRITICAL();
	return have;
}

void bc_rt_send_evt(uint8_t cmd_id, uint32_t param)
{
	QueueHandle_t q = main_GetMODHandle(E_QUEUE_BALANCE);
	cmdqu_t m = { 0 };

	if (!q)
		return;
	m.ip_id = IP_BALANCE;
	m.cmd_id = cmd_id & 0x7F;
	m.param_ptr = param;
	/* outbound events are recognised by cmd_id >= 0x40 in the comm task */
	xQueueSend(q, &m, 0);
}

static void forward_to_linux(const cmdqu_t *m)
{
	QueueHandle_t q = main_GetMODHandle(E_QUEUE_CMDQU);
	cmdqu_t out = *m;

	if (!q)
		return;
	out.ip_id = IP_BALANCE;
	out.block = 0;
	out.resv.valid.linux_valid = 0;
	out.resv.valid.rtos_valid = 1;
	/* CMDQU task's default branch posts it to Linux through the mailbox */
	xQueueSend(q, &out, 0);
}

static void handle_cmd(const cmdqu_t *m)
{
	volatile bc_shm_t *shm = BC_SHM();
	uint32_t p = m->param_ptr;

	bc_event_push(shm, bc_now_us(), BC_EVT_LOG_CMD, m->cmd_id, (int32_t)p);
	switch (m->cmd_id) {
	case BC_CMD_PING:
		bc_rt_send_evt(BC_EVT_PONG, p);
		break;
	case BC_CMD_ARM:
		__atomic_fetch_or(&g_events, BC_EV_ARM, __ATOMIC_ACQ_REL);
		break;
	case BC_CMD_DISARM:
		__atomic_fetch_or(&g_events, BC_EV_DISARM, __ATOMIC_ACQ_REL);
		break;
	case BC_CMD_ESTOP:
		__atomic_fetch_or(&g_events, BC_EV_ESTOP, __ATOMIC_ACQ_REL);
		break;
	case BC_CMD_SET_TARGET:
		taskENTER_CRITICAL();
		g_speed_mmps = bc_unpack_speed(p);
		g_turn_mradps = bc_unpack_turn(p);
		g_target_ms = bc_now_ms();
		taskEXIT_CRITICAL();
		break;
	case BC_CMD_HEARTBEAT:
		taskENTER_CRITICAL();
		g_hb_counter = p;
		g_hb_ms = bc_now_ms();
		taskEXIT_CRITICAL();
		break;
	case BC_CMD_CALIBRATE:
		__atomic_store_n(&g_calib_req, p, __ATOMIC_RELEASE);
		break;
	case BC_CMD_APPLY_PARAMS: {
		bc_params_t np;
		uint32_t seq = 0;
		int rc = bc_param_blk_read(&shm->param, &np, &seq);
		int bad = 0;

		if (rc == 0)
			bad = bc_params_validate(&np);
		if (rc != 0) {
			bad = 100 + (-rc);	/* 101 torn, 102 crc/size/version */
		} else if (!bad) {
			taskENTER_CRITICAL();
			g_pending_params = np;
			g_pending_seq = seq;
			g_have_params = 1;
			taskEXIT_CRITICAL();
		}
		bc_event_push(shm, bc_now_us(), BC_EVT_LOG_PARAMS, (uint16_t)bad, (int32_t)seq);
		bc_rt_send_evt(BC_EVT_PARAMS_ACK, (uint32_t)bad);
		break;
	}
	default:
		__atomic_fetch_add(&shm->status.lost_cmds, 1u, __ATOMIC_RELAXED);
		break;
	}
}

void prvBalanceCommTask(void *pvParameters)
{
	QueueHandle_t q = main_GetMODHandle(E_QUEUE_BALANCE);
	cmdqu_t m;

	(void)pvParameters;
	for (;;) {
		if (xQueueReceive(q, &m, portMAX_DELAY) != pdTRUE)
			continue;
		if (m.cmd_id >= 0x40)
			forward_to_linux(&m);		/* internal request: send to Linux */
		else
			handle_cmd(&m);			/* from Linux */
	}
}
