"""Telemetry logger rotation, video profiles / adaptive rate, bcd integration."""
import json
import os
import tempfile
import unittest

import common  # noqa: F401  (sets sys.path)
import bc_log
import bc_video as V
import bcd as B
import bc_shm as S


def read_lines(path):
    with open(path) as f:
        return f.read().splitlines()


class TestLogger(unittest.TestCase):
    def test_writes_json_lines(self):
        with tempfile.TemporaryDirectory() as d:
            lg = bc_log.TelemLogger(d)
            lg.write({"a": 1})
            lg.write({"a": 2})
            lg.close()
            lines = read_lines(os.path.join(d, "telem.jsonl"))
            self.assertEqual([json.loads(x)["a"] for x in lines], [1, 2])

    def test_rotation_bounds_total_size_and_keeps_newest(self):
        with tempfile.TemporaryDirectory() as d:
            lg = bc_log.TelemLogger(d, max_bytes=200, keep=2)
            for i in range(200):
                lg.write({"i": i, "pad": "x" * 10})
            lg.close()
            names = sorted(os.listdir(d))
            self.assertEqual(names, ["telem.jsonl", "telem.jsonl.1", "telem.jsonl.2"])
            total = sum(os.path.getsize(os.path.join(d, n)) for n in names)
            self.assertLessEqual(total, 3 * 200)
            last = json.loads(read_lines(os.path.join(d, "telem.jsonl"))[-1])
            self.assertEqual(last["i"], 199)

    def test_appends_to_existing_file_and_never_raises(self):
        with tempfile.TemporaryDirectory() as d:
            lg = bc_log.TelemLogger(d)
            lg.write({"a": 1})
            lg.close()
            lg2 = bc_log.TelemLogger(d)
            lg2.write({"a": 2})
            lg2.write({"bad": object()})          # not serialisable
            self.assertEqual(lg2.errors, 1)
            lg2.close()
            self.assertEqual(len(read_lines(os.path.join(d, "telem.jsonl"))), 2)


class TestVideoProfiles(unittest.TestCase):
    def test_save_load_roundtrip_and_env(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "v.json")
            self.assertEqual(V.load(p), V.DEFAULT)
            V.save("fpv-low", p)
            self.assertEqual(V.load(p), "fpv-low")
            self.assertIn("BC_VIDEO_WIDTH=640", V.env_lines("fpv-low"))
            with open(p, "w") as f:
                f.write("{broken")
            self.assertEqual(V.load(p), V.DEFAULT)

    def test_steps_down_fast_up_slowly(self):
        a = V.AdaptiveRate("fpv-hq")
        self.assertIsNone(a.update(-80))
        self.assertEqual(a.update(-80), "fpv-med")
        self.assertIsNone(a.update(-80))
        self.assertEqual(a.update(-80), "fpv-low")
        for _ in range(4):
            self.assertIsNone(a.update(-80))          # floor reached
        for i in range(9):
            self.assertIsNone(a.update(-50), i)
        self.assertEqual(a.update(-50), "fpv-med")

    def test_hysteresis_between_thresholds_keeps_profile(self):
        a = V.AdaptiveRate("fpv-med")
        for _ in range(50):
            self.assertIsNone(a.update(-70))

    def test_lost_telemetry_counts_as_bad_link(self):
        a = V.AdaptiveRate("fpv-hq")
        a.update(-50, 0)
        a.update(-50, 100)
        self.assertEqual(a.update(-50, 200), "fpv-med")

    def test_manual_choice_stops_adaptation_until_auto(self):
        a = V.AdaptiveRate("fpv-hq")
        a.set_manual("fpv-med")
        for _ in range(10):
            self.assertIsNone(a.update(-90))
        a.set_auto()
        a.update(-90)
        self.assertEqual(a.update(-90), "fpv-low")
        with self.assertRaises(ValueError):
            a.set_manual("nope")

    def test_mjpeg_is_not_adapted(self):
        a = V.AdaptiveRate("mjpeg")
        for _ in range(10):
            self.assertIsNone(a.update(-90))


class TestBcdIntegration(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        fd, self.path = tempfile.mkstemp(suffix=".shm")
        os.close(fd)
        common.tool("shm_tool", "init", self.path)
        self.shm = S.Shm.open_file(self.path)
        self.vpath = os.path.join(self.tmp.name, "v.json")
        self.bcd = B.Bcd(self.shm, S.RecordingMailbox(), None, None, lambda m: None,
                         video_path=self.vpath)
        self.sent = []
        self.bcd.listeners.append(self.sent.append)

    def tearDown(self):
        self.bcd.stop()
        os.unlink(self.path)
        self.tmp.cleanup()

    def test_video_message_sets_profile_and_persists(self):
        r = self.bcd.handle_message({"t": "video", "profile": "fpv-low"})
        self.assertEqual((r["t"], r["profile"], r["auto"]), ("video", "fpv-low", False))
        self.assertEqual(V.load(self.vpath), "fpv-low")
        self.assertEqual(self.bcd.handle_message({"t": "video", "profile": "bogus"})["t"], "error")
        self.assertTrue(self.bcd.handle_message({"t": "video", "profile": "auto"})["auto"])

    def test_adaptive_change_is_broadcast_and_stored(self):
        self.bcd.video.profile = "fpv-hq"
        self.bcd.adapt_video(-85, 0)
        self.bcd.adapt_video(-85, 0)
        msgs = [json.loads(m) for m in self.sent]
        self.assertEqual(msgs[-1]["t"], "video")
        self.assertEqual(msgs[-1]["profile"], "fpv-med")
        self.assertEqual(V.load(self.vpath), "fpv-med")

    def test_telemetry_json_has_odometry(self):
        self.bcd.telem = S.TelemReader(self.shm)
        common.tool("shm_tool", "telem", self.path, 5)
        recs = self.bcd.poll_telemetry()
        j = B.Bcd.tel_json(recs[-1])
        self.assertAlmostEqual(j["x"], 4 * 3 / 1000.0)
        self.assertAlmostEqual(j["psi"], ((4 % 100) - 50) / 1000.0)

    def test_logger_receives_every_record_and_calib_in_status(self):
        lg = bc_log.TelemLogger(self.tmp.name)
        self.bcd.logger = lg
        self.bcd.telem = S.TelemReader(self.shm)
        common.tool("shm_tool", "telem", self.path, 3)
        for r in self.bcd.poll_telemetry():
            self.bcd.logger.write(r)
        lg.close()
        n = len(read_lines(os.path.join(self.tmp.name, "telem.jsonl")))
        self.assertEqual(n, 3)
        common.tool("shm_tool", "calib", self.path)
        self.assertEqual(self.shm.read_calib()["trim_deg"], 1.75)


if __name__ == "__main__":
    unittest.main()
