"""DMP image packing, hand-over to the RTOS (real C reader) and bcd/bcctl integration."""
import contextlib
import io
import os
import re
import tempfile
import unittest

import common
import bc_dmp as D
import bc_layout as L
import bc_shm as S
import bcctl
import bcd as B

# a stand-in for the library source: comments contain hex numbers on purpose
def fake_cpp(data):
    rows = []
    for i in range(0, len(data), 16):
        rows.append(", ".join("0x%02X" % b for b in data[i:i + 16]) + ",")
        if i == 32:
            rows.append("// change 2742 from 0xD8 to 0x20 - not image data")
        if i == 64:
            rows.append("/* bank # 1 0xAA 0xBB */")
    return ("#define MPU6050_DMP_CODE_SIZE 3062\nconst unsigned char dmpMemory[MPU6050_DMP_CODE_SIZE] PROGMEM = {\n"
            + "\n".join(rows) + "\n};\nint other[] = {0x11, 0x22};\n")


def image():
    return bytes((i * 31 + 7) & 0xFF for i in range(D.IMAGE_SIZE))


def run_ctl(path, *argv):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        rc = bcctl.main(["--shm-file", path, "--no-mailbox", *argv])
    return rc, out.getvalue(), err.getvalue()


class TestPacking(unittest.TestCase):
    def test_extract_ignores_comments_and_other_arrays(self):
        img = image()
        self.assertEqual(D.extract_from_cpp(fake_cpp(img)), img)

    def test_wrong_source_and_wrong_size_rejected(self):
        with self.assertRaises(D.DmpError):
            D.extract_from_cpp("int x[] = {0x01};")
        with self.assertRaises(D.DmpError):
            D.validate(image()[:100])

    def test_unknown_checksum_needs_force(self):
        img = image()
        with self.assertRaises(D.DmpError):
            D.validate(img)                       # not the known InvenSense image
        self.assertEqual(D.validate(img, force=True), S.crc32(img))

    def test_known_checksum_constant_is_a_crc32(self):
        self.assertTrue(0 < D.IMAGE_CRC32 < 2 ** 32)


class TestShmHandOver(unittest.TestCase):
    def setUp(self):
        fd, self.path = tempfile.mkstemp(suffix=".shm")
        os.close(fd)
        common.tool("shm_tool", "init", self.path)
        self.shm = S.Shm.open_file(self.path)

    def tearDown(self):
        os.unlink(self.path)

    def test_python_write_is_accepted_by_the_c_reader(self):
        self.assertIn("rc=-1", common.tool("shm_tool", "dmpread", self.path))     # nothing yet
        D.write_image(self.shm, image())
        out = common.tool("shm_tool", "dmpread", self.path)
        self.assertIn("rc=0 size=%d crc=%08x" % (D.IMAGE_SIZE, S.crc32(image())), out)

    def test_c_write_is_accepted_by_python_reader(self):
        fd, f = tempfile.mkstemp()
        os.write(fd, image())
        os.close(fd)
        try:
            common.tool("shm_tool", "dmpwrite", self.path, f)
        finally:
            os.unlink(f)
        kind, data = D.read_image(self.shm)
        self.assertEqual((kind, data), (L.DMP_KIND_612, image()))

    def test_corruption_is_detected_by_both_sides(self):
        D.write_image(self.shm, image())
        self.shm.m[L.OFF_DMP + 16 + 100] ^= 0xFF
        self.assertIsNone(D.read_image(self.shm))
        self.assertIn("rc=-2", common.tool("shm_tool", "dmpread", self.path))

    def test_oversize_image_refused(self):
        with self.assertRaises(D.DmpError):
            D.write_image(self.shm, bytes(L.DMP_DATA_MAX + 1))

    def test_status_decoding(self):
        st = self.shm.status()
        self.assertEqual(D.state_name(st), "OFF")
        st["dmp_info"] = 1 | (1234 << 8)
        self.assertEqual((D.state_name(st), D.fallbacks(st)), ("RUNNING", 1234))
        st["dmp_info"] = 9
        self.assertEqual(D.state_name(st), "?")


class TestBcdAndCtl(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        fd, self.path = tempfile.mkstemp(suffix=".shm")
        os.close(fd)
        common.tool("shm_tool", "init", self.path)
        self.shm = S.Shm.open_file(self.path)
        self.img = os.path.join(self.tmp.name, "dmp.bin")
        with open(self.img, "wb") as f:
            f.write(image())
        self.logs = []

    def tearDown(self):
        os.unlink(self.path)
        self.tmp.cleanup()

    def bcd(self, path):
        return B.Bcd(self.shm, S.RecordingMailbox(), None, None, self.logs.append, dmp_path=path)

    def test_unknown_checksum_image_is_not_offered(self):
        d = self.bcd(self.img)                     # synthetic image: crc differs from the real one
        self.assertFalse(d.sync_dmp())
        self.assertTrue(any("crc32" in m for m in self.logs))
        self.assertIsNone(D.read_image(self.shm))

    def test_missing_file_is_silent_and_no_dmp_path_is_a_noop(self):
        self.assertFalse(self.bcd(os.path.join(self.tmp.name, "none.bin")).sync_dmp())
        self.assertFalse(self.bcd(None).sync_dmp())
        self.assertEqual(self.logs, [])

    def test_image_offered_once_per_rtos_boot(self):
        d = self.bcd(self.img)
        orig = D.IMAGE_CRC32
        D.IMAGE_CRC32 = S.crc32(image())           # accept the synthetic image for this test
        try:
            self.assertTrue(d.sync_dmp())
            self.assertFalse(d.sync_dmp())         # same boot: nothing to do
            self.assertIsNotNone(D.read_image(self.shm))
            self.shm.m[L.OFF_DMP:L.OFF_DMP + 4096] = bytes(4096)   # RTOS restarted: window zeroed
            self.shm.set_u32(12, self.shm.header()["boot_count"] + 1)   # boot_count is word 3
            self.assertTrue(d.sync_dmp())
            self.assertIsNotNone(D.read_image(self.shm))
        finally:
            D.IMAGE_CRC32 = orig

    def test_dmp_info_and_telemetry_flag(self):
        d = self.bcd(None)
        self.assertEqual(d.dmp_info(), {"state": "OFF", "fallbacks": 0, "image": False})
        rec = {k: 0 for k in L.TELEM_FIELDS}
        rec["flags"] = 16
        self.assertTrue(B.Bcd.tel_json(rec)["dmp"])
        rec["flags"] = 15
        self.assertFalse(B.Bcd.tel_json(rec)["dmp"])

    def test_bcctl_pack_load_status(self):
        src = os.path.join(self.tmp.name, "lib.cpp")
        dst = os.path.join(self.tmp.name, "packed.bin")
        with open(src, "w") as f:
            f.write(fake_cpp(image()))
        rc, _, err = run_ctl(self.path, "dmp-pack", src, dst)
        self.assertEqual(rc, 1)                    # not the known image
        self.assertIn("crc32", err)
        rc, out, _ = run_ctl(self.path, "dmp-pack", src, dst, "--force")
        self.assertEqual(rc, 0)
        self.assertIn("3062 bytes", out)
        rc, _, err = run_ctl(self.path, "dmp-load", dst)
        self.assertEqual(rc, 1)                    # load re-checks the checksum
        rc, out, _ = run_ctl(self.path, "dmp-load", dst, "--force")
        self.assertEqual(rc, 0)
        self.assertIn("est_mode 2", out)
        rc, out, _ = run_ctl(self.path, "dmp-status")
        self.assertIn('"image_in_shm": true', out)
        self.assertIn('"state": "OFF"', out)


if __name__ == "__main__":
    unittest.main()
