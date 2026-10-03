import contextlib
import io
import os
import subprocess
import tempfile
import unittest

import common
import bc_layout as L
import bc_shm as S
import bcctl


def run(path, *argv):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        rc = bcctl.main(["--shm-file", path, "--no-mailbox", *argv])
    return rc, out.getvalue(), err.getvalue()


class TestBcctl(unittest.TestCase):
    def setUp(self):
        fd, self.path = tempfile.mkstemp(suffix=".shm")
        os.close(fd)
        common.tool("shm_tool", "init", self.path)

    def tearDown(self):
        os.unlink(self.path)

    def test_status_and_watch(self):
        common.tool("shm_tool", "telem", self.path, 6)
        rc, out, _ = run(self.path, "status")
        self.assertEqual(rc, 0)
        self.assertIn('"telem_head": 6', out)
        self.assertIn("tool-build", out)
        # watch reads only new records: add some after attaching
        shm = S.Shm.open_file(self.path)
        rd = S.TelemReader(shm)
        common.tool("shm_tool", "telem", self.path, 2)
        self.assertEqual(len(rd.poll()), 2)

    def test_events(self):
        common.tool("shm_tool", "event", self.path, 3)
        rc, out, _ = run(self.path, "events")
        self.assertEqual(rc, 0)
        self.assertEqual(len(out.strip().splitlines()), 3)

    def test_get_requires_rtos_params_then_works(self):
        rc, _, err = run(self.path, "get")
        self.assertEqual(rc, 1)
        self.assertIn("not published", err)
        shm = S.Shm.open_file(self.path)
        shm.write_params(S.defaults())
        common.tool("shm_tool", "apply", self.path)
        rc, out, _ = run(self.path, "get", "a_kp")
        self.assertEqual((rc, out.strip()), (0, "15.0"))
        rc, out, _ = run(self.path, "get")
        self.assertIn("v_kp", out)
        rc, _, err = run(self.path, "get", "nope")
        self.assertEqual(rc, 2)

    def test_set_rejects_bad_values_and_names(self):
        rc, _, err = run(self.path, "set", "a_kp", "9999")
        self.assertEqual(rc, 2)
        self.assertIn("rejected", err)
        rc, _, err = run(self.path, "set", "nope", "1")
        self.assertEqual(rc, 2)


class TestScripts(unittest.TestCase):
    def test_shell_scripts_parse(self):
        for name in ("bc-start.sh", "bc-net.sh", "bc-video.sh"):
            r = subprocess.run(["sh", "-n", str(common.BC_DIR / name)], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, name + ": " + r.stderr)

    def test_python_modules_compile(self):
        for name in ("bcd.py", "bcctl.py", "bc_camera.py", "bc_shm.py", "bc_layout.py"):
            r = subprocess.run(["python3", "-m", "py_compile", str(common.BC_DIR / name)],
                               capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, name + ": " + r.stderr)

    def test_duo_init_launches_balance_services_and_not_pwm_module(self):
        text = (common.REPO / "device/generic/rootfs_overlay/duos/mnt/system/duo-init.sh").read_text()
        self.assertIn("bc-start.sh", text)
        active = [ln for ln in text.splitlines() if "cv181x_pwm.ko" in ln and not ln.lstrip().startswith("#")]
        self.assertEqual(active, [], "Linux must not load the PWM module the RTOS owns")

    def test_dts_release_rtos_owned_pads(self):
        for dts in (common.REPO / "build/boards/cv181x").glob("*duos*/dts_*/*.dts"):
            text = dts.read_text()
            for node in ("&i2c4", "&spi3"):
                i = text.index(node + " {")
                self.assertIn('status = "disabled"', text[i:i + 60], f"{dts.name}: {node}")

    def test_web_ui_present_and_uses_protocol(self):
        html = (common.BC_DIR / "web/index.html").read_text()
        for needle in ("WebSocket", "'ctl'", "'estop'", "t:'param'"):
            self.assertIn(needle, html)


if __name__ == "__main__":
    unittest.main()
