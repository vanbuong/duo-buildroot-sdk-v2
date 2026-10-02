/*
 * Test helper that plays the RTOS side of the shared window on a file, using
 * the real bc_shm.h code, so the Python side is checked against it.
 *   shm_tool init FILE
 *   shm_tool telem FILE N          push N telemetry records (pattern below)
 *   shm_tool event FILE N          push N events
 *   shm_tool apply FILE            read+validate the Linux param block, write the echo
 *   shm_tool status FILE           print status words
 *   shm_tool heartbeat FILE        read the Linux heartbeat
 */
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include "bc_shm.h"

static volatile bc_shm_t *map(const char *path, int create)
{
	int fd = open(path, O_RDWR | (create ? O_CREAT : 0), 0644);
	void *p;

	if (fd < 0) { perror("open"); exit(2); }
	if (create && ftruncate(fd, BC_SHM_SIZE)) { perror("ftruncate"); exit(2); }
	p = mmap(NULL, BC_SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (p == MAP_FAILED) { perror("mmap"); exit(2); }
	close(fd);
	return p;
}

int main(int argc, char **argv)
{
	volatile bc_shm_t *s;
	unsigned i, n;

	if (argc < 3)
		return 2;
	s = map(argv[2], !strcmp(argv[1], "init"));
	if (!strcmp(argv[1], "init")) {
		bc_shm_init(s, 3, "tool-build");
	} else if (!strcmp(argv[1], "telem") && argc > 3) {
		n = (unsigned)atoi(argv[3]);
		for (i = 0; i < n; i++) {
			bc_telem_t t;

			memset(&t, 0, sizeof(t));
			t.t_us = 1000u * i;
			t.pitch_cdeg = (int16_t)(i % 3000) - 1500;
			t.pitch_acc_cdeg = (int16_t)i;
			t.gyro_y_cdps = -(int16_t)i;
			t.speed_l_mmps = 100;
			t.speed_r_mmps = -100;
			t.motor_l_pm = 250;
			t.motor_r_pm = -250;
			t.enc_l = (int32_t)(i * 10);
			t.enc_r = -(int32_t)(i * 10);
			t.vbat_mv = 11800;
			t.exec_us = 400;
			t.period_us = 5000;
			t.state = 4;
			t.fault = 0;
			bc_telem_push(s, &t);
		}
		s->status.state = 4;
		s->status.rtos_heartbeat = 77;
	} else if (!strcmp(argv[1], "event") && argc > 3) {
		n = (unsigned)atoi(argv[3]);
		for (i = 0; i < n; i++)
			bc_event_push(s, 100 * i, BC_EVT_LOG_STATE, (uint16_t)(i % 8), -(int32_t)i);
	} else if (!strcmp(argv[1], "apply")) {
		bc_params_t p;
		uint32_t seq = 0;
		int rc = bc_param_blk_read(&s->param, &p, &seq);
		int bad = rc ? 100 - rc : bc_params_validate(&p);

		if (!rc && !bad) {
			bc_param_echo_write(s, &p, 0, seq);
		} else {
			bc_params_t d;

			bc_params_default(&d);
			bc_param_echo_write(s, &d, (uint32_t)bad, seq);
		}
		printf("rc=%d bad=%d seq=%u a_kp=%g imu_sign=%d\n", rc, bad, seq, rc ? 0.0 : (double)p.a_kp,
		       rc ? 0 : p.imu_sign);
	} else if (!strcmp(argv[1], "status")) {
		printf("magic=%08x heartbeat=%u telem_head=%u event_head=%u state=%u\n", s->hdr.magic,
		       s->status.rtos_heartbeat, s->status.telem_head, s->status.event_head, s->status.state);
	} else if (!strcmp(argv[1], "heartbeat")) {
		printf("%u\n", s->lnx.linux_heartbeat);
	} else {
		return 2;
	}
	return 0;
}
