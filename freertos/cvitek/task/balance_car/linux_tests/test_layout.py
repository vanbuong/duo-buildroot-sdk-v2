"""The generated Python layout must match the C compiler's view exactly."""
import struct
import unittest

import common  # noqa: F401  (sets sys.path)
import bc_layout as L


def c_layout():
    out = {}
    params = []
    for line in common.tool("layout_dump").splitlines():
        parts = line.split()
        if parts[0] == "param":
            params.append((parts[1], int(parts[2]), parts[3], float(parts[4]), float(parts[5])))
        else:
            out[parts[0]] = int(parts[1])
    return out, params


class LayoutTest(unittest.TestCase):
    def setUp(self):
        self.c, self.params = c_layout()

    def test_window_offsets(self):
        c = self.c
        self.assertEqual(c["shm_size"], L.SHM_SIZE)
        self.assertEqual((c["off_hdr"], c["off_status"], c["off_linux"]),
                         (L.OFF_HDR, L.OFF_STATUS, L.OFF_LINUX))
        self.assertEqual((c["off_param"], c["off_echo"], c["off_calib"]),
                         (L.OFF_PARAM, L.OFF_ECHO, L.OFF_CALIB))
        self.assertEqual((c["off_telem"], c["off_events"]), (L.OFF_TELEM, L.OFF_EVENTS))
        self.assertEqual((c["telem_slots"], c["event_slots"]), (L.TELEM_SLOTS, L.EVENT_SLOTS))
        self.assertEqual(c["magic"], L.SHM_MAGIC)

    def test_record_sizes_and_fields(self):
        c = self.c
        self.assertEqual(c["sizeof_telem"], struct.calcsize(L.TELEM_FMT))
        self.assertEqual(c["sizeof_event"], struct.calcsize(L.EVENT_FMT))
        # field offsets inside the telemetry record
        sizes = struct.calcsize
        pos = sizes("<II")
        self.assertEqual(c["telem_t_us"], 4)
        self.assertEqual(c["telem_pitch"], pos)
        self.assertEqual(c["telem_enc_l"], pos + 24)
        self.assertEqual(c["telem_vbat"], pos + 24 + 8)
        self.assertEqual(c["telem_state"], pos + 24 + 8 + 6)
        self.assertEqual(c["telem_crc"], 60)
        self.assertEqual(c["event_val"], 12)

    def test_status_fields(self):
        c = self.c
        idx = {n: i * 4 for i, n in enumerate(L.STATUS_FIELDS)}
        self.assertEqual(c["status_heartbeat"], idx["rtos_heartbeat"])
        self.assertEqual(c["status_telem_head"], idx["telem_head"])
        self.assertEqual(c["status_event_head"], idx["event_head"])
        self.assertEqual(c["status_lost_cmds"], idx["lost_cmds"])

    def test_parameter_table_identical(self):
        self.assertEqual(self.c["param_count"], L.PARAM_COUNT)
        self.assertEqual(self.c["sizeof_params"], struct.calcsize(L.PARAM_FMT))
        self.assertEqual(len(self.params), len(L.PARAMS))
        for (cn, off, typ, lo, hi), (pn, ptyp, _d, plo, phi) in zip(self.params, L.PARAMS):
            self.assertEqual(cn, pn)
            self.assertEqual(typ, ptyp, cn)
            self.assertAlmostEqual(lo, plo, places=4, msg=cn)
            self.assertAlmostEqual(hi, phi, places=4, msg=cn)
        for i, (cn, off, *_rest) in enumerate(self.params):
            self.assertEqual(off, 4 * i, cn)

    def test_param_blocks(self):
        self.assertEqual(self.c["param_blk_size"], struct.calcsize(L.PARAM_BLK_FMT))
        self.assertEqual(self.c["echo_size"], struct.calcsize(L.ECHO_FMT))
        self.assertEqual(self.c["off_param_p"], 16)
        self.assertEqual(self.c["off_echo_p"], 16)

    def test_protocol_ids(self):
        self.assertEqual(self.c["ip_id"], L.BC_IP_ID)
        self.assertEqual(self.c["cmd_set_target"], L.BC_CMD_SET_TARGET)
        self.assertEqual(self.c["evt_state"], L.BC_EVT_STATE)


if __name__ == "__main__":
    unittest.main()
