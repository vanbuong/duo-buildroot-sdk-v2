import base64
import hashlib
import json
import os
import socket
import struct
import tempfile
import threading
import time
import unittest

import common
import bc_layout as L
import bc_shm as S
import bcd as B


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class CToolMailbox(S.RecordingMailbox):
    """APPLY_PARAMS is answered by the real RTOS C code (shm_tool apply)."""

    def __init__(self, path):
        super().__init__()
        self.path = path
        self.answer = True

    def send(self, cmd_id, param=0):
        super().send(cmd_id, param)
        if cmd_id == L.BC_CMD_APPLY_PARAMS and self.answer:
            common.tool("shm_tool", "apply", self.path)


class BcdBase(unittest.TestCase):
    def setUp(self):
        fd, self.path = tempfile.mkstemp(suffix=".shm")
        os.close(fd)
        common.tool("shm_tool", "init", self.path)
        self.shm = S.Shm.open_file(self.path)
        self.mbox = CToolMailbox(self.path)
        self.clock = Clock()
        self.emergencies = 0
        self.logs = []

        def emergency():
            self.emergencies += 1
        self.bcd = B.Bcd(self.shm, self.mbox, None, emergency, self.logs.append, self.clock)

    def tearDown(self):
        self.bcd.stop()
        os.unlink(self.path)

    def sent(self, cmd):
        return [p for c, p in self.mbox.sent if c == cmd]


class TestHeartbeatAndTargets(BcdBase):
    def test_heartbeat_ticks(self):
        for _ in range(3):
            self.bcd.tick_heartbeat()
        self.assertEqual(self.sent(L.BC_CMD_HEARTBEAT), [1, 2, 3])
        self.assertEqual(self.shm.u32(L.OFF_LINUX), 3)
        self.assertEqual(self.sent(L.BC_CMD_SET_TARGET), [0, 0, 0])

    def test_target_clamped_and_forwarded(self):
        self.bcd.set_target(5.0, -9.0, immediate=True)        # way above limits
        p = self.sent(L.BC_CMD_SET_TARGET)[-1]
        self.assertEqual(p & 0xFFFF, 500)                     # v_max 0.5 m/s
        self.assertEqual((p >> 16), (-2000) & 0xFFFF)         # w_max 2 rad/s
        self.bcd.set_target(0.25, 0.5)
        self.bcd.tick_heartbeat()
        self.assertEqual(self.sent(L.BC_CMD_SET_TARGET)[-1], S.pack_target(250, 500))

    def test_deadman_zeroes_target_when_client_silent(self):
        self.bcd.set_target(0.3, 0.4, immediate=True)
        self.clock.t += 0.2
        self.bcd.tick_heartbeat()
        self.assertEqual(self.sent(L.BC_CMD_SET_TARGET)[-1], S.pack_target(300, 400))
        self.clock.t += 0.2                                    # > 300 ms since last ctl
        self.bcd.tick_heartbeat()
        self.assertEqual(self.sent(L.BC_CMD_SET_TARGET)[-1], 0)
        self.assertEqual(self.bcd.target, (0, 0))

    def test_commands(self):
        self.bcd.arm()
        self.bcd.disarm()
        self.bcd.estop()
        self.bcd.calibrate("gyro")
        self.bcd.calibrate("trim")
        self.assertEqual(self.mbox.sent[0], (L.BC_CMD_ARM, 0))
        self.assertEqual(self.mbox.sent[1], (L.BC_CMD_DISARM, 0))
        self.assertIn((L.BC_CMD_ESTOP, 0), self.mbox.sent)
        self.assertEqual(self.sent(L.BC_CMD_CALIBRATE), [L.BC_CALIB_GYRO, L.BC_CALIB_TRIM])
        with self.assertRaises(ValueError):
            self.bcd.calibrate("nope")

    def test_estop_zeroes_target_before_sending_estop(self):
        self.bcd.set_target(0.4, 0.0, immediate=True)
        self.bcd.estop()
        i_stop = self.mbox.sent.index((L.BC_CMD_ESTOP, 0))
        zero = [i for i, m in enumerate(self.mbox.sent) if m == (L.BC_CMD_SET_TARGET, 0)]
        self.assertTrue(zero and zero[-1] < i_stop)


class TestParams(BcdBase):
    def test_push_waits_for_real_rtos_echo(self):
        ok, msg = self.bcd.set_param("a_kp", 18.0)
        self.assertTrue(ok, msg)
        self.assertAlmostEqual(self.shm.read_echo()["params"]["a_kp"], 18.0, places=5)
        ok, msg = self.bcd.set_param("a_kd", 1.0)
        self.assertTrue(ok, msg)
        p = self.shm.read_echo()["params"]                 # earlier change kept
        self.assertAlmostEqual(p["a_kp"], 18.0, places=5)
        self.assertAlmostEqual(p["a_kd"], 1.0, places=5)

    def test_invalid_values_never_reach_the_rtos(self):
        ok, msg = self.bcd.set_param("a_kp", 1000.0)
        self.assertFalse(ok)
        self.assertIn("a_kp", msg)
        ok, msg = self.bcd.set_param("imu_sign", 0)
        self.assertFalse(ok)
        ok, msg = self.bcd.set_param("bogus", 1)
        self.assertFalse(ok)
        self.assertEqual(self.sent(L.BC_CMD_APPLY_PARAMS), [])

    def test_timeout_when_rtos_does_not_answer(self):
        self.mbox.answer = False
        real = time.monotonic
        self.bcd.clock = real
        ok, msg = self.bcd.push_params({"a_kp": 17.0}, wait_s=0.1)
        self.assertFalse(ok)
        self.assertIn("did not confirm", msg)

    def test_save_and_load_roundtrip(self):
        d = tempfile.mkdtemp()
        self.bcd.params_path = os.path.join(d, "bc_params.json")
        self.assertIsNone(self.bcd.load_params())          # nothing saved yet
        self.bcd.set_param("trim_deg", 1.5)
        self.bcd.save_params()
        with open(self.bcd.params_path) as f:
            saved = json.load(f)
        self.assertAlmostEqual(saved["trim_deg"], 1.5, places=5)
        self.bcd.set_param("trim_deg", -3.0)
        self.assertTrue(self.bcd.load_params())
        self.assertAlmostEqual(self.shm.read_echo()["params"]["trim_deg"], 1.5, places=5)


class TestLiveness(BcdBase):
    def test_emergency_when_rtos_heartbeat_freezes(self):
        common.tool("shm_tool", "telem", self.path, 1)     # sets heartbeat 77
        self.bcd.tick_heartbeat()
        self.assertTrue(self.bcd.rtos_alive)
        self.clock.t += 0.2
        self.bcd.tick_heartbeat()
        self.assertEqual(self.emergencies, 0)              # not frozen long enough
        self.clock.t += 0.2
        self.bcd.tick_heartbeat()
        self.assertEqual(self.emergencies, 1)              # >300 ms
        self.assertFalse(self.bcd.rtos_alive)
        self.clock.t += 1.0
        self.bcd.tick_heartbeat()
        self.assertEqual(self.emergencies, 1)              # fired once, not repeatedly
        self.shm.set_u32(L.OFF_STATUS, 78)                 # RTOS came back
        self.bcd.tick_heartbeat()
        self.assertTrue(self.bcd.rtos_alive)
        self.clock.t += 0.5
        self.bcd.tick_heartbeat()
        self.assertEqual(self.emergencies, 2)              # re-armed

    def test_no_emergency_if_rtos_never_ran(self):
        self.bcd.tick_heartbeat()                          # heartbeat stays 0
        self.clock.t += 5
        self.bcd.tick_heartbeat()
        self.assertEqual(self.emergencies, 0)


class TestTelemetry(BcdBase):
    def test_poll_and_json(self):
        self.bcd.telem = S.TelemReader(self.shm)
        self.bcd.events = S.EventReader(self.shm)
        common.tool("shm_tool", "telem", self.path, 10)
        recs = self.bcd.poll_telemetry()
        self.assertEqual(len(recs), 10)
        j = B.Bcd.tel_json(recs[-1], lost=2, rssi=-55.0, alive=True)
        self.assertEqual(j["t"], "tel")
        self.assertAlmostEqual(j["pitch"], (9 % 3000 - 1500) / 100.0)
        self.assertEqual(j["state"], "BALANCING")
        self.assertEqual(j["fault"], "NONE")
        self.assertAlmostEqual(j["vbat"], 11.8)
        self.assertEqual(j["lost"], 2)
        json.dumps(j)

    def test_event_json_state(self):
        ev = {"t_us": 5, "type": L.EVT_LOG_STATE, "arg": 6, "val": 2}
        j = B.Bcd.event_json(ev)
        self.assertEqual((j["t"], j["state"], j["fault"]), ("state", "FAULT", "LIFT"))
        ev = {"t_us": 5, "type": L.EVT_LOG_PARAMS, "arg": 0, "val": 8}
        self.assertEqual(B.Bcd.event_json(ev)["t"], "param_ack")


class TestMessages(BcdBase):
    def test_handle_message(self):
        h = self.bcd.handle_message
        self.assertIsNone(h({"t": "ctl", "v": 0.2, "w": 0.1}))
        self.assertEqual(self.sent(L.BC_CMD_SET_TARGET)[-1], S.pack_target(200, 100))
        h({"t": "arm"})
        self.assertIn((L.BC_CMD_ARM, 0), self.mbox.sent)
        r = h({"t": "param", "k": "a_kp", "v": 19.0})
        self.assertTrue(r["ok"])
        r = h({"t": "param", "k": "a_kp", "v": 9999})
        self.assertFalse(r["ok"])
        self.assertEqual(h({"t": "ping", "seq": 5}), {"t": "pong", "seq": 5})
        self.assertEqual(h({"t": "nope"})["t"], "error")
        self.assertIn("a_kp", h({"t": "params"})["params"])


# ---- websocket ---------------------------------------------------------------
def masked_frame(opcode, payload):
    mask = os.urandom(4)
    n = len(payload)
    head = bytes([0x80 | opcode])
    if n < 126:
        head += bytes([0x80 | n])
    else:
        head += bytes([0x80 | 126]) + struct.pack("!H", n)
    return head + mask + bytes(b ^ mask[i & 3] for i, b in enumerate(payload))


class WsClient:
    def __init__(self, port):
        self.s = socket.create_connection(("127.0.0.1", port), timeout=3)
        key = base64.b64encode(os.urandom(16)).decode()
        self.s.sendall(("GET /ws HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                        "Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n" % key).encode())
        data = b""
        while b"\r\n\r\n" not in data:
            data += self.s.recv(1024)
        self.head, _, self.buf = data.partition(b"\r\n\r\n")
        want = base64.b64encode(hashlib.sha1((key + B.GUID).encode()).digest())
        assert want in self.head, self.head
        self.buf = bytearray(self.buf)

    def send(self, obj):
        self.s.sendall(masked_frame(1, json.dumps(obj).encode()))

    def recv(self):
        while True:
            f = B.ws_parse(self.buf)
            if f:
                return f
            self.buf += self.s.recv(4096)

    def close(self):
        self.s.close()


class TestServer(BcdBase):
    def setUp(self):
        super().setUp()
        self.web = tempfile.mkdtemp()
        with open(os.path.join(self.web, "index.html"), "w") as f:
            f.write("<html>robot</html>")
        self.server = B.WsServer(self.bcd, host="127.0.0.1", port=0, static_dir=self.web)
        self.server.start()

    def tearDown(self):
        self.server.stop()
        super().tearDown()

    def wait_for(self, pred, t=2.0):
        end = time.time() + t
        while time.time() < end:
            if pred():
                return True
            time.sleep(0.01)
        return False

    def test_rfc6455_accept_example(self):
        key = "dGhlIHNhbXBsZSBub25jZQ=="
        acc = base64.b64encode(hashlib.sha1((key + B.GUID).encode()).digest()).decode()
        self.assertEqual(acc, "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=")

    def test_websocket_control_roundtrip(self):
        c = WsClient(self.server.port)
        c.send({"t": "ctl", "v": 0.2, "w": 0.0})
        self.assertTrue(self.wait_for(lambda: S.pack_target(200, 0) in self.sent(L.BC_CMD_SET_TARGET)))
        c.send({"t": "ping", "seq": 9})
        op, payload = c.recv()
        self.assertEqual((op, json.loads(payload)), (1, {"t": "pong", "seq": 9}))
        c.send({"t": "param", "k": "a_kd", "v": 1.2})
        op, payload = c.recv()
        self.assertTrue(json.loads(payload)["ok"])
        self.bcd.broadcast(json.dumps({"t": "tel", "pitch": 1.0}))
        op, payload = c.recv()
        self.assertEqual(json.loads(payload)["t"], "tel")
        c.s.sendall(masked_frame(9, b"hi"))                 # ping -> pong
        op, payload = c.recv()
        self.assertEqual((op, payload), (0xA, b"hi"))
        c.send({"bad": 1})
        c.s.sendall(masked_frame(1, b"{not json"))
        replies = [json.loads(c.recv()[1])["t"] for _ in range(2)]
        self.assertEqual(replies, ["error", "error"])
        c.close()

    def test_client_disconnect_leaves_deadman_to_zero_target(self):
        c = WsClient(self.server.port)
        c.send({"t": "ctl", "v": 0.3, "w": 0.0})
        self.assertTrue(self.wait_for(lambda: self.bcd.target != (0, 0)))
        c.close()
        self.assertTrue(self.wait_for(lambda: not self.bcd.listeners))
        self.clock.t += 0.5
        self.bcd.tick_heartbeat()
        self.assertEqual(self.sent(L.BC_CMD_SET_TARGET)[-1], 0)

    def test_http_endpoints(self):
        def get(path):
            s = socket.create_connection(("127.0.0.1", self.server.port), timeout=3)
            s.sendall(("GET %s HTTP/1.1\r\nHost: x\r\n\r\n" % path).encode())
            data = b""
            while True:
                chunk = s.recv(4096)
                if not chunk:
                    break
                data += chunk
            s.close()
            return data
        self.assertIn(b"200 OK", get("/api/status"))
        self.assertIn(b"BOOT", get("/api/status"))
        self.assertIn(b"<html>robot</html>", get("/"))
        self.assertIn(b"404", get("/nothing"))


class TestFraming(unittest.TestCase):
    def test_lengths_and_masking(self):
        for n in (0, 5, 125, 126, 300, 65535, 70000):
            payload = bytes(range(256)) * (n // 256 + 1)
            payload = payload[:n]
            buf = bytearray(masked_frame(1, payload) if n < 65536 else B.ws_frame(1, payload))
            op, got = B.ws_parse(buf)
            self.assertEqual((op, got), (1, payload))
            self.assertEqual(len(buf), 0)

    def test_incomplete_frame_returns_none_and_keeps_buffer(self):
        full = masked_frame(1, b"hello world")
        for cut in range(len(full)):
            buf = bytearray(full[:cut])
            self.assertIsNone(B.ws_parse(buf))
            self.assertEqual(len(buf), cut)

    def test_two_frames_in_one_buffer(self):
        buf = bytearray(masked_frame(1, b"a") + masked_frame(1, b"bc"))
        self.assertEqual(B.ws_parse(buf), (1, b"a"))
        self.assertEqual(B.ws_parse(buf), (1, b"bc"))
        self.assertIsNone(B.ws_parse(buf))


if __name__ == "__main__":
    unittest.main()
