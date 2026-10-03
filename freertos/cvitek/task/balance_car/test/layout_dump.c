/* Prints the C compiler's view of the shared layout for the Python cross-check. */
#include <stdio.h>
#include <stddef.h>
#include "bc_shm.h"
#include "bc_proto.h"

#define P(name, v) printf("%s %lu\n", name, (unsigned long)(v))

int main(void)
{
	unsigned i;

	P("shm_size", sizeof(bc_shm_t));
	P("off_hdr", offsetof(bc_shm_t, hdr));
	P("off_status", offsetof(bc_shm_t, status));
	P("off_linux", offsetof(bc_shm_t, lnx));
	P("off_param", offsetof(bc_shm_t, param));
	P("off_echo", offsetof(bc_shm_t, echo));
	P("off_calib", offsetof(bc_shm_t, calib));
	P("off_telem", offsetof(bc_shm_t, telem));
	P("off_events", offsetof(bc_shm_t, events));
	P("telem_slots", BC_TELEM_SLOTS);
	P("event_slots", BC_EVENT_SLOTS);
	P("sizeof_telem", sizeof(bc_telem_t));
	P("sizeof_event", sizeof(bc_event_t));
	P("sizeof_params", sizeof(bc_params_t));
	P("param_count", bc_param_count);
	P("param_blk_size", sizeof(bc_param_blk_t));
	P("echo_size", sizeof(bc_param_echo_t));
	P("off_param_p", offsetof(bc_param_blk_t, p));
	P("off_echo_p", offsetof(bc_param_echo_t, p));
	P("telem_t_us", offsetof(bc_telem_t, t_us));
	P("telem_pitch", offsetof(bc_telem_t, pitch_cdeg));
	P("telem_enc_l", offsetof(bc_telem_t, enc_l));
	P("telem_vbat", offsetof(bc_telem_t, vbat_mv));
	P("telem_state", offsetof(bc_telem_t, state));
	P("telem_psi", offsetof(bc_telem_t, psi_mrad));
	P("telem_x", offsetof(bc_telem_t, x_mm));
	P("off_dmp", offsetof(bc_shm_t, dmp));
	P("dmp_data", offsetof(bc_dmp_blk_t, data));
	P("dmp_data_max", BC_DMP_DATA_MAX);
	P("dmp_kind_612", BC_DMP_KIND_612);
	P("status_dmp_info", offsetof(bc_shm_status_t, dmp_info));
	P("status_imu_variant", offsetof(bc_shm_status_t, imu_variant));
	P("telem_crc", offsetof(bc_telem_t, crc));
	P("status_heartbeat", offsetof(bc_shm_status_t, rtos_heartbeat));
	P("status_telem_head", offsetof(bc_shm_status_t, telem_head));
	P("status_event_head", offsetof(bc_shm_status_t, event_head));
	P("status_lost_cmds", offsetof(bc_shm_status_t, lost_cmds));
	P("event_val", offsetof(bc_event_t, val));
	P("magic", BC_SHM_MAGIC);
	P("ip_id", BC_IP_ID);
	P("cmd_set_target", BC_CMD_SET_TARGET);
	P("evt_state", BC_EVT_STATE);
	for (i = 0; i < bc_param_count; i++)
		printf("param %s %u %c %g %g\n", bc_param_table[i].name, bc_param_table[i].offset,
		       bc_param_table[i].type, (double)bc_param_table[i].min,
		       (double)bc_param_table[i].max);
	return 0;
}
