import json
import os
import tempfile
import unittest

import common  # noqa: F401
import bc_camera as C


class Args:
    def __init__(self, data, **kw):
        self.force = kw.get("force", "auto")
        self.prefer = kw.get("prefer", "j1")
        self.conf = os.path.join(data, "bc_camera.conf")
        self.data = data
        self.dry_run = kw.get("dry_run", False)


def make_data():
    d = tempfile.mkdtemp()
    for ini in ("sensor_cfg_GC2083.ini", "sensor_cfg_OV5647_J1.ini", "sensor_cfg_OV5647_J2.ini"):
        with open(os.path.join(d, ini), "w") as f:
            f.write("; " + ini + "\n")
    return d


def reader_for(present):
    """present: set of (bus, addr) that answer with the right chip id."""
    ids = {(3, 0x37): bytes([0x20, 0x83]), (3, 0x36): bytes([0x56, 0x47]),
           (2, 0x36): bytes([0x56, 0x47]), (2, 0x10): bytes([0x02, 0x19])}

    def read(bus, addr, reg, regbytes, n=2, dev_fmt=None):
        if (bus, addr) in present:
            return ids[(bus, addr)]
        raise OSError("no ack")
    return read


class FakeGpio:
    def __init__(self):
        self.calls = []

    def set(self, num, value):
        self.calls.append((num, value))


class TestChoose(unittest.TestCase):
    def test_table(self):
        gc = ("GC2083", "sensor_cfg_GC2083.ini")
        ov = ("OV5647", "sensor_cfg_OV5647_J2.ini")
        self.assertEqual(C.choose({"j1": gc, "j2": None})[0], "j1")
        self.assertEqual(C.choose({"j1": None, "j2": ov})[0], "j2")
        self.assertEqual(C.choose({"j1": gc, "j2": ov}, "j1")[0], "j1")
        self.assertEqual(C.choose({"j1": gc, "j2": ov}, "j2")[0], "j2")
        c, s, ini, why = C.choose({"j1": None, "j2": None}, "j2")
        self.assertEqual((c, s, ini), ("j2", "OV5647", "sensor_cfg_OV5647_J2.ini"))
        self.assertIn("none detected", why)
        self.assertEqual(C.choose({"j1": gc, "j2": ov}, "j1", forced="j2")[0], "j2")


class TestSelect(unittest.TestCase):
    def test_only_j1_gc2083(self):
        d = make_data()
        g = FakeGpio()
        r = C.select(Args(d), reader_for({(3, 0x37)}), g)
        self.assertEqual((r["connector"], r["sensor"]), ("j1", "GC2083"))
        with open(os.path.join(d, "sensor_cfg.ini")) as f:
            self.assertIn("GC2083", f.read())
        self.assertIn((C.CONNECTORS["j2"]["reset"], 0), g.calls)     # unused sensor held in reset
        self.assertEqual(g.calls[-2:], [(C.CONNECTORS["j1"]["reset"], 1), (C.CONNECTORS["j2"]["reset"], 0)])

    def test_only_j2_ov5647(self):
        d = make_data()
        r = C.select(Args(d), reader_for({(2, 0x36)}), FakeGpio())
        self.assertEqual((r["connector"], r["sensor"]), ("j2", "OV5647"))
        with open(os.path.join(d, "sensor_cfg.ini")) as f:
            self.assertIn("OV5647_J2", f.read())

    def test_both_present_prefers_conf(self):
        d = make_data()
        with open(os.path.join(d, "bc_camera.conf"), "w") as f:
            json.dump({"preferred": "j2"}, f)
        r = C.select(Args(d), reader_for({(3, 0x37), (2, 0x36)}), FakeGpio())
        self.assertEqual(r["connector"], "j2")

    def test_forced_skips_probe(self):
        d = make_data()

        def exploding(*a, **k):
            raise AssertionError("must not probe when forced")
        r = C.select(Args(d, force="j2"), exploding, FakeGpio())
        self.assertEqual(r["connector"], "j2")
        self.assertNotIn("error", r)

    def test_none_detected_uses_preferred(self):
        d = make_data()
        r = C.select(Args(d, prefer="j1"), reader_for(set()), FakeGpio())
        self.assertEqual(r["connector"], "j1")
        self.assertIn("none detected", r["why"])

    def test_imx219_has_no_driver(self):
        d = make_data()
        r = C.select(Args(d, force="auto"), reader_for({(2, 0x10)}), FakeGpio())
        # IMX219 on J2 is probed after OV5647; only IMX219 answers
        self.assertEqual(r["connector"], "j2")
        self.assertEqual(r["sensor"], "IMX219")
        self.assertIn("error", r)
        self.assertFalse(os.path.exists(os.path.join(d, "sensor_cfg.ini")))

    def test_missing_ini_reported(self):
        d = tempfile.mkdtemp()
        r = C.select(Args(d), reader_for({(3, 0x37)}), None)
        self.assertIn("missing", r["error"])

    def test_dry_run_changes_nothing(self):
        d = make_data()
        g = FakeGpio()
        r = C.select(Args(d, dry_run=True), reader_for({(3, 0x37)}), g)
        self.assertNotIn("error", r)
        self.assertFalse(os.path.exists(os.path.join(d, "sensor_cfg.ini")))

    def test_gpio_sysfs_writes(self):
        root = tempfile.mkdtemp()
        os.makedirs(os.path.join(root, "gpio482"))
        open(os.path.join(root, "export"), "w").close()
        C.Gpio(root).set(482, 0)
        with open(os.path.join(root, "gpio482", "value")) as f:
            self.assertEqual(f.read(), "0")
        with open(os.path.join(root, "gpio482", "direction")) as f:
            self.assertEqual(f.read(), "out")

    def test_sensor_ini_names_exist_in_the_sdk_overlay(self):
        data = common.REPO / "device/generic/rootfs_overlay/duos/mnt/data"
        for conn in C.CONNECTORS.values():
            for _s, _a, _r, _rb, _e, ini in conn["sensors"]:
                if ini:
                    self.assertTrue((data / ini).exists(), ini)


if __name__ == "__main__":
    unittest.main()
