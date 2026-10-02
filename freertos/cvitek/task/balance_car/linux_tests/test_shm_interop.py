"""Python Shm <-> real C bc_shm.h code on the same file."""
import os
import tempfile
import unittest

import common
import bc_layout as L
import bc_shm as S


class InteropTest(unittest.TestCase):
    def setUp(self):
        fd, self.path = tempfile.mkstemp(suffix=".shm")
        os.close(fd)
        common.tool("shm_tool", "init", self.path)
        self.shm = S.Shm.open_file(self.path)

    def tearDown(self):
        os.unlink(self.path)

    def test_header_written_by_c(self):
        self.assertTrue(self.shm.ready())
        h = self.shm.header()
        self.assertEqual(h["boot_count"], 3)
        self.assertEqual(h["build_id"], "tool-build")

    def test_telemetry_from_c(self):
        common.tool("shm_tool", "telem", self.path, 40)
        st = self.shm.status()
        self.assertEqual(st["telem_head"], 40)
        self.assertEqual(st["rtos_heartbeat"], 77)
        self.assertEqual(st["state_name"], "BALANCING")
        for n in (0, 17, 39):
            rc, r = self.shm.read_telem(n)
            self.assertEqual(rc, 0)
            self.assertEqual(r["t_us"], 1000 * n)
            self.assertEqual(r["enc_l"], 10 * n)
            self.assertEqual(r["enc_r"], -10 * n)
            self.assertEqual(r["pitch_cdeg"], n % 3000 - 1500)
            self.assertEqual(r["vbat_mv"], 11800)
            self.assertEqual(r["period_us"], 5000)
        self.assertEqual(self.shm.read_telem(40)[0], 1)

    def test_c_crc_validates_in_python_and_corruption_detected(self):
        common.tool("shm_tool", "telem", self.path, 3)
        self.assertEqual(self.shm.read_telem(1)[0], 0)
        off = L.OFF_TELEM + 64 + 32              # enc_l of record 1
        self.shm.m[off] ^= 0x01
        self.assertEqual(self.shm.read_telem(1)[0], -2)

    def test_ring_overrun_reported(self):
        common.tool("shm_tool", "telem", self.path, 300)
        self.assertEqual(self.shm.read_telem(0)[0], -1)
        self.assertEqual(self.shm.read_telem(299)[0], 0)
        rd = S.TelemReader(self.shm)
        rd.next = 0
        recs = rd.poll(limit=1000)
        self.assertGreater(rd.lost, 0)
        self.assertEqual(recs[-1]["enc_l"], 2990)
        self.assertEqual([r["t_us"] for r in recs], sorted(r["t_us"] for r in recs))

    def test_events_from_c(self):
        common.tool("shm_tool", "event", self.path, 5)
        rd = S.EventReader(self.shm, from_start=True)
        evs = rd.poll()
        self.assertEqual(len(evs), 5)
        self.assertEqual(evs[3]["arg"], 3)
        self.assertEqual(evs[3]["val"], -3)
        self.assertEqual(evs[3]["type"], L.EVT_LOG_STATE)

    def test_params_python_to_c_and_back(self):
        p = S.defaults()
        p.update(a_kp=21.5, a_kd=1.1, imu_sign=-1, enc_cpr=2400.0)
        seq = self.shm.write_params(p)
        out = common.tool("shm_tool", "apply", self.path)
        self.assertIn("rc=0 bad=0", out)
        self.assertIn("a_kp=21.5", out)
        self.assertIn("imu_sign=-1", out)
        e = self.shm.read_echo()
        self.assertEqual(e["applied_seq"], seq)
        self.assertEqual(e["applied_count"], 1)
        self.assertEqual(e["last_result"], 0)
        self.assertAlmostEqual(e["params"]["a_kp"], 21.5, places=5)
        self.assertEqual(e["params"]["imu_sign"], -1)
        self.assertAlmostEqual(e["params"]["enc_cpr"], 2400.0, places=3)

    def test_c_rejects_corrupt_param_block(self):
        self.shm.write_params(S.defaults())
        self.shm.m[L.OFF_PARAM + 16 + 8] ^= 0x40      # flip a bit in the payload
        out = common.tool("shm_tool", "apply", self.path)
        self.assertNotIn("bad=0", out)
        self.assertNotEqual(self.shm.read_echo()["last_result"], 0)

    def test_python_validation_matches_c_ranges(self):
        for name, typ, _d, lo, hi in L.PARAMS:
            p = S.defaults()
            p[name] = hi + 1000
            self.assertIsNotNone(S.validate_params(p), name)
            p[name] = lo - 1000
            self.assertIsNotNone(S.validate_params(p), name)
        bad = S.defaults()
        bad["a_kp"] = float("nan")
        self.assertIsNotNone(S.validate_params(bad))
        with self.assertRaises(S.ShmError):
            self.shm.write_params({"a_kp": 1000.0})
        with self.assertRaises(S.ShmError):
            self.shm.write_params({"nope": 1.0})

    def test_linux_heartbeat_visible_to_c(self):
        for _ in range(3):
            self.shm.bump_heartbeat()
        self.assertEqual(common.tool("shm_tool", "heartbeat", self.path).strip(), "3")

    def test_mailbox_packing_matches_proto(self):
        raw = S.Mailbox.pack(L.BC_CMD_SET_TARGET, S.pack_target(300, -500))
        self.assertEqual(len(raw), 8)
        self.assertEqual(raw[0], L.BC_IP_ID)
        self.assertEqual(raw[1], L.BC_CMD_SET_TARGET)
        self.assertEqual(int.from_bytes(raw[4:], "little"), 0xFE0C012C)
        self.assertEqual(S.RTOS_CMDQU_SEND, 0x40087201)


if __name__ == "__main__":
    unittest.main()
