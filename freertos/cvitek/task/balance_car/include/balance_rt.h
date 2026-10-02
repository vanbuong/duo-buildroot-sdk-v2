/*
 * Internal interface between the balance control task (balance_main.c) and
 * the Linux communication task (balance_comm.c). Single-writer rules:
 * the comm task writes commands/params/events, the control task consumes them.
 */
#ifndef BALANCE_RT_H
#define BALANCE_RT_H

#include <stdint.h>
#include "arch_helpers.h"

/* cache maintenance for the RTOS side of the shared window (bc_shm.h rules) */
#define BC_SHM_CLEAN(p, n)	clean_dcache_range((uintptr_t)(p), (size_t)(n))
#define BC_SHM_INV(p, n)	inv_dcache_range((uintptr_t)(p), (size_t)(n))

#include "bc_params.h"
#include "bc_shm.h"

/* ---- RTOS shared-memory window (linker symbol, see cv181x_lscript.ld) ---- */
extern char _bc_shm_base[];
#define BC_SHM()	((volatile bc_shm_t *)(void *)_bc_shm_base)

/* ---- time ----------------------------------------------------------------- */
uint32_t bc_now_us(void);
uint32_t bc_now_ms(void);

/* ---- comm task -> control task -------------------------------------------- */
/* Consume pending BC_EV_* bits (atomically clears them). */
uint32_t bc_rt_take_events(void);
/* Latest command snapshot; copies under a short critical section. */
void bc_rt_get_cmd(int32_t *speed_mmps, int32_t *turn_mradps, uint32_t *target_ms,
		   uint32_t *hb_counter, uint32_t *hb_ms);
/* A validated parameter set is waiting. Returns 1 and fills p/seq if so. */
int bc_rt_take_params(bc_params_t *p, uint32_t *seq);
/* Calibration request: 0 none, else BC_CALIB_*; clears the request. */
uint32_t bc_rt_take_calib(void);

/* ---- control task -> comm task -------------------------------------------- */
/* Queue a BC_EVT_* mailbox message for Linux (never blocks, may drop). */
void bc_rt_send_evt(uint8_t cmd_id, uint32_t param);

void prvBalanceCommTask(void *pvParameters);

#endif
