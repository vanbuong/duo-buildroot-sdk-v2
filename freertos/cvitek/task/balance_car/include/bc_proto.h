/*
 * Linux <-> RTOS protocol: mailbox command ids and payload packing.
 * Shared (include verbatim) by the RTOS and the Linux side.
 * docs/balance_car/01 section 6.1.
 */
#ifndef BC_PROTO_H
#define BC_PROTO_H

#include <stdint.h>

/* ip_id for cmdqu_t (see rtos_cmdqu.h: IP_BALANCE) */
#define BC_IP_ID		8

/* Linux -> RTOS */
#define BC_CMD_PING		0x01	/* param: nonce                          */
#define BC_CMD_ARM		0x02
#define BC_CMD_DISARM		0x03
#define BC_CMD_SET_TARGET	0x04	/* param: bc_pack_target()               */
#define BC_CMD_APPLY_PARAMS	0x05	/* param: param block seq                */
#define BC_CMD_HEARTBEAT	0x06	/* param: counter                        */
#define BC_CMD_CALIBRATE	0x07	/* param: 1 gyro, 2 level trim           */
#define BC_CMD_ESTOP		0x08

/* RTOS -> Linux */
#define BC_EVT_STATE		0x40	/* param: bc_pack_state()                */
#define BC_EVT_PONG		0x41
#define BC_EVT_PARAMS_ACK	0x42	/* param: 0 ok, else 1 + bad field index */
#define BC_EVT_CALIB_DONE	0x43	/* param: 0 ok, else error               */
#define BC_EVT_HEARTBEAT	0x44
#define BC_EVT_FAULT		0x45

#define BC_CALIB_GYRO		1
#define BC_CALIB_TRIM		2

static inline uint32_t bc_pack_target(int32_t speed_mmps, int32_t turn_mradps)
{
	if (speed_mmps > 32767) speed_mmps = 32767;
	if (speed_mmps < -32768) speed_mmps = -32768;
	if (turn_mradps > 32767) turn_mradps = 32767;
	if (turn_mradps < -32768) turn_mradps = -32768;
	return ((uint32_t)(uint16_t)(int16_t)turn_mradps << 16) |
	       (uint32_t)(uint16_t)(int16_t)speed_mmps;
}

static inline int32_t bc_unpack_speed(uint32_t p) { return (int16_t)(p & 0xFFFFu); }
static inline int32_t bc_unpack_turn(uint32_t p) { return (int16_t)(p >> 16); }

static inline uint32_t bc_pack_state(unsigned state, unsigned fault, unsigned flags)
{
	return (state & 0xFFu) | ((fault & 0xFFu) << 8) | ((flags & 0xFFFFu) << 16);
}
static inline unsigned bc_unpack_state(uint32_t p) { return p & 0xFFu; }
static inline unsigned bc_unpack_fault(uint32_t p) { return (p >> 8) & 0xFFu; }
static inline unsigned bc_unpack_flags(uint32_t p) { return p >> 16; }

#endif
