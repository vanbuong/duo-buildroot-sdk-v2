#!/usr/bin/env python3
"""bcd - balance control daemon (Linux side).

Responsibilities (docs/balance_car/01 and 02):
  * heartbeat to the RTOS (shm counter + mailbox) at 10 Hz
  * teleoperation: WebSocket JSON control with a dead-man timer
  * telemetry from the shared-memory ring, broadcast to clients and logged
  * run-time parameters: validate, push, wait for the RTOS echo, persist
  * emergency path: RTOS heartbeat frozen -> clear TB6612 STBY directly
Nothing here is needed to stay upright; the RTOS only depends on the heartbeat
to zero its targets.
"""
import argparse
import base64
import hashlib
import json
import os
import socket
import struct
import threading
import time

import bc_layout as L
import bc_shm as S

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
DEADMAN_S = 0.3
HB_PERIOD_S = 0.1
RTOS_DEAD_S = 0.3
TARGET_PERIOD_S = 0.05

GPIOB_BASE = 0x03021000
STBY_BIT = 15                      # BC_PIN_STBY = GPIOB_PIN(15)


# ---------------------------------------------------------------------------
class DevMemGpio:
    """Emergency STBY clear through /dev/mem (single documented exception to
    'RTOS owns its pins': one-way, towards the safe state)."""

    def __init__(self, path="/dev/mem"):
        import mmap
        fd = os.open(path, os.O_RDWR | os.O_SYNC)
        self.m = mmap.mmap(fd, 0x1000, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE,
                           offset=GPIOB_BASE)
        os.close(fd)

    def clear_stby(self):
        dr = struct.unpack_from("<I", self.m, 0)[0]
        struct.pack_into("<I", self.m, 0, dr & ~(1 << STBY_BIT))


class Bcd:
    def __init__(self, shm, mbox, params_path=None, emergency=None, log=print,
                 clock=time.monotonic):
        self.shm = shm
        self.mbox = mbox
        self.params_path = params_path
        self.emergency = emergency          # callable() or None
        self.log = log
        self.clock = clock
        self.lock = threading.RLock()
        self.target = (0, 0)                # mm/s, mrad/s
        self.target_time = None
        self.hb_count = 0
        self.rtos_hb = None
        self.rtos_hb_time = clock()
        self.rtos_alive = None
        self.emergency_fired = False
        self.params = S.defaults()
        self.telem = S.TelemReader(shm) if shm.ready() else None
        self.events = S.EventReader(shm) if shm.ready() else None
        self.latest = None
        self.listeners = []                 # callables(json_str)
        self.stop_flag = threading.Event()
        self.threads = []
        self.link_rssi = None

    # -- commands ---------------------------------------------------------
    def arm(self):
        self.mbox.send(L.BC_CMD_ARM, 0)

    def disarm(self):
        self.mbox.send(L.BC_CMD_DISARM, 0)

    def estop(self):
        self.set_target(0.0, 0.0, immediate=True)
        self.mbox.send(L.BC_CMD_ESTOP, 0)

    def calibrate(self, what):
        kind = {"gyro": L.BC_CALIB_GYRO, "trim": L.BC_CALIB_TRIM}.get(what)
        if kind is None:
            raise ValueError("calibrate: gyro|trim")
        self.mbox.send(L.BC_CMD_CALIBRATE, kind)

    def set_target(self, v_mps, w_rads, immediate=False):
        """Clamp to the configured limits; remembered until the dead-man expires."""
        vmax = self.params.get("v_max", 0.5)
        wmax = self.params.get("w_max", 2.0)
        v = max(-vmax, min(vmax, float(v_mps)))
        w = max(-wmax, min(wmax, float(w_rads)))
        with self.lock:
            self.target = (int(round(v * 1000)), int(round(w * 1000)))
            self.target_time = self.clock()
        if immediate:
            self.mbox.send(L.BC_CMD_SET_TARGET, S.pack_target(*self.target))

    def ping(self, nonce=1):
        self.mbox.send(L.BC_CMD_PING, nonce)

    # -- parameters -----------------------------------------------------------
    def current_params(self):
        return self.shm.current_params()

    def push_params(self, changes=None, wait_s=1.0):
        """Merge `changes` into the current set, write, apply, wait for the echo.
        Returns (ok, message)."""
        with self.lock:
            full = self.current_params()
            full.update(changes or {})
            err = S.validate_params(S.fill_defaults(full))
            if err:
                return False, err
            seq = self.shm.write_params(full)
            before = self.shm.read_echo()
            base = before["applied_count"] if before else 0
            self.mbox.send(L.BC_CMD_APPLY_PARAMS, seq)
            deadline = self.clock() + wait_s
            while self.clock() < deadline:
                e = self.shm.read_echo()
                if e and e["applied_seq"] == seq and e["applied_count"] > base:
                    self.params = dict(e["params"])
                    return True, "applied seq %d" % seq
                time.sleep(0.01)
            return False, "RTOS did not confirm seq %d" % seq

    def set_param(self, name, value):
        if S.param_index(name) < 0:
            return False, "unknown parameter " + name
        return self.push_params({name: value})

    def save_params(self):
        if not self.params_path:
            return
        tmp = self.params_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.current_params(), f, indent=1, sort_keys=True)
        os.replace(tmp, self.params_path)

    def load_params(self):
        if self.params_path and os.path.exists(self.params_path):
            with open(self.params_path) as f:
                saved = json.load(f)
            ok, msg = self.push_params(saved)
            self.log("params from %s: %s" % (self.params_path, msg))
            return ok
        return None

    # -- periodic work -----------------------------------------------------------
    def tick_heartbeat(self):
        """One 10 Hz step: heartbeat out, RTOS liveness in, target refresh."""
        self.hb_count = self.shm.bump_heartbeat()
        self.mbox.send(L.BC_CMD_HEARTBEAT, self.hb_count)
        self.check_rtos()
        self.refresh_target()

    def refresh_target(self):
        with self.lock:
            tt = self.target_time
            if tt is not None and self.clock() - tt > DEADMAN_S:
                self.target = (0, 0)        # dead-man: client went quiet
                self.target_time = None
            tgt = self.target
        self.mbox.send(L.BC_CMD_SET_TARGET, S.pack_target(*tgt))

    def check_rtos(self):
        hb = self.shm.status()["rtos_heartbeat"]
        t = self.clock()
        if self.rtos_hb is None or hb != self.rtos_hb:
            if self.rtos_hb is not None and not self.rtos_alive:
                self.log("RTOS heartbeat resumed")
            self.rtos_hb, self.rtos_hb_time = hb, t
            self.rtos_alive = True
            self.emergency_fired = False
        elif t - self.rtos_hb_time > RTOS_DEAD_S and self.rtos_alive is not None:
            if self.rtos_alive:
                self.log("RTOS heartbeat frozen for %.0f ms" % ((t - self.rtos_hb_time) * 1000))
            self.rtos_alive = False
            if not self.emergency_fired and self.emergency and hb != 0:
                self.emergency_fired = True
                try:
                    self.emergency()
                    self.log("emergency: TB6612 STBY cleared")
                except Exception as exc:         # noqa: BLE001
                    self.log("emergency path failed: %s" % exc)

    def poll_telemetry(self):
        if self.telem is None:
            if self.shm.ready():
                self.telem = S.TelemReader(self.shm)
                self.events = S.EventReader(self.shm)
            return []
        recs = self.telem.poll()
        for r in recs:
            self.latest = r
        for ev in (self.events.poll() if self.events else []):
            self.broadcast(json.dumps(self.event_json(ev)))
        return recs

    @staticmethod
    def tel_json(r, lost=0, rssi=None, alive=True):
        names, faults = L.STATE_NAMES, L.FAULT_NAMES
        return {"t": "tel", "ts": r["t_us"],
                "pitch": r["pitch_cdeg"] / 100.0, "pitch_acc": r["pitch_acc_cdeg"] / 100.0,
                "gyro": r["gyro_y_cdps"] / 100.0, "vl": r["speed_l_mmps"] / 1000.0,
                "vr": r["speed_r_mmps"] / 1000.0,
                "u": (r["motor_l_pm"] + r["motor_r_pm"]) / 20.0,
                "state": names[r["state"]] if r["state"] < len(names) else "?",
                "fault": faults[r["fault"]] if r["fault"] < len(faults) else "?",
                "period_us": r["period_us"], "exec_us": r["exec_us"],
                "vbat": r["vbat_mv"] / 1000.0, "rssi": rssi, "lost": lost,
                "rtos": alive}

    @staticmethod
    def event_json(ev):
        d = {"t": "event", "ts": ev["t_us"], "type": ev["type"], "arg": ev["arg"], "val": ev["val"]}
        if ev["type"] == L.EVT_LOG_STATE:
            d.update(t="state", state=L.STATE_NAMES[ev["arg"]] if ev["arg"] < len(L.STATE_NAMES) else "?",
                     fault=L.FAULT_NAMES[ev["val"]] if 0 <= ev["val"] < len(L.FAULT_NAMES) else "?")
        elif ev["type"] == L.EVT_LOG_PARAMS:
            d.update(t="param_ack", result=ev["arg"], seq=ev["val"])
        return d

    def read_rssi(self):
        try:
            with open("/proc/net/wireless") as f:
                for line in f.readlines()[2:]:
                    parts = line.split()
                    return float(parts[3].rstrip("."))
        except (OSError, IndexError, ValueError):
            pass
        return None

    def broadcast(self, text):
        for fn in list(self.listeners):
            try:
                fn(text)
            except Exception:                  # noqa: BLE001
                self.listeners.remove(fn)

    # -- threads ---------------------------------------------------------------
    def run_forever(self):
        def hb_loop():
            while not self.stop_flag.wait(HB_PERIOD_S):
                try:
                    self.tick_heartbeat()
                except Exception as exc:       # noqa: BLE001
                    self.log("heartbeat error: %s" % exc)

        def tel_loop():
            n = 0
            while not self.stop_flag.wait(0.02):
                recs = self.poll_telemetry()
                for r in recs:
                    n += 1
                    if n % 3 == 0:            # 50 Hz ring -> ~17 Hz to clients
                        lost = self.telem.lost if self.telem else 0
                        if n % 60 == 0:
                            self.link_rssi = self.read_rssi()
                        self.broadcast(json.dumps(self.tel_json(
                            r, lost, self.link_rssi, bool(self.rtos_alive))))

        for fn in (hb_loop, tel_loop):
            th = threading.Thread(target=fn, daemon=True)
            th.start()
            self.threads.append(th)

    def stop(self):
        self.stop_flag.set()
        for th in self.threads:
            th.join(timeout=1.0)

    # -- client message handling (shared by WebSocket and tests) ------------------
    def handle_message(self, msg):
        """Process one decoded JSON message; returns a reply dict or None."""
        t = msg.get("t")
        if t == "ctl":
            self.set_target(msg.get("v", 0.0), msg.get("w", 0.0), immediate=True)
            return None
        if t == "arm":
            self.arm()
        elif t == "disarm":
            self.disarm()
        elif t == "estop":
            self.estop()
        elif t == "param":
            ok, text = self.set_param(msg.get("k", ""), msg.get("v"))
            return {"t": "param_result", "k": msg.get("k"), "ok": ok, "msg": text}
        elif t == "save":
            self.save_params()
            return {"t": "saved"}
        elif t == "calib":
            self.calibrate(msg.get("what", ""))
        elif t == "params":
            return {"t": "params", "params": self.current_params()}
        elif t == "ping":
            return {"t": "pong", "seq": msg.get("seq")}
        else:
            return {"t": "error", "msg": "unknown message type %r" % (t,)}
        return None


# ---------------------------------------------------------------------------
# minimal HTTP + WebSocket (RFC 6455) server, no third-party modules
class WsServer:
    def __init__(self, bcd, host="0.0.0.0", port=8080, static_dir=None):
        self.bcd = bcd
        self.static_dir = static_dir
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.listen(8)
        self.port = self.sock.getsockname()[1]
        self.stop_flag = threading.Event()

    def serve_forever(self):
        self.sock.settimeout(0.2)
        while not self.stop_flag.is_set():
            try:
                conn, _ = self.sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self.handle, args=(conn,), daemon=True).start()

    def start(self):
        th = threading.Thread(target=self.serve_forever, daemon=True)
        th.start()
        return th

    def stop(self):
        self.stop_flag.set()
        try:
            self.sock.close()
        except OSError:
            pass

    # -- connection ------------------------------------------------------------
    @staticmethod
    def _read_headers(conn):
        data = b""
        while b"\r\n\r\n" not in data:
            chunk = conn.recv(2048)
            if not chunk:
                return None, b""
            data += chunk
            if len(data) > 16384:
                return None, b""
        head, _, rest = data.partition(b"\r\n\r\n")
        lines = head.decode("latin-1").split("\r\n")
        hdrs = {}
        for ln in lines[1:]:
            if ":" in ln:
                k, v = ln.split(":", 1)
                hdrs[k.strip().lower()] = v.strip()
        return (lines[0], hdrs), rest

    def handle(self, conn):
        try:
            req, rest = self._read_headers(conn)
            if not req:
                return
            line, hdrs = req
            parts = line.split()
            path = parts[1] if len(parts) > 1 else "/"
            if hdrs.get("upgrade", "").lower() == "websocket" and path.split("?")[0] == "/ws":
                self.websocket(conn, hdrs, rest)
            else:
                self.http(conn, path)
        except OSError:
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def http(self, conn, path):
        path = path.split("?")[0]
        if path == "/api/status":
            body = json.dumps({"status": self.bcd.shm.status(), "header": self.bcd.shm.header(),
                               "rtos_alive": self.bcd.rtos_alive}).encode()
            ctype = "application/json"
        elif path in ("/", "/index.html") and self.static_dir:
            try:
                with open(os.path.join(self.static_dir, "index.html"), "rb") as f:
                    body = f.read()
                ctype = "text/html; charset=utf-8"
            except OSError:
                conn.sendall(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n")
                return
        else:
            conn.sendall(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n")
            return
        conn.sendall(("HTTP/1.1 200 OK\r\nContent-Type: %s\r\nContent-Length: %d\r\n"
                      "Connection: close\r\n\r\n" % (ctype, len(body))).encode() + body)

    def websocket(self, conn, hdrs, rest):
        key = hdrs.get("sec-websocket-key", "")
        accept = base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()
        conn.sendall(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
                      "Connection: Upgrade\r\nSec-WebSocket-Accept: %s\r\n\r\n" % accept).encode())
        send_lock = threading.Lock()

        def send_text(text):
            with send_lock:
                conn.sendall(ws_frame(0x1, text.encode()))

        self.bcd.listeners.append(send_text)
        buf = bytearray(rest)
        conn.settimeout(1.0)
        try:
            while not self.stop_flag.is_set():
                frame = ws_parse(buf)
                if frame is None:
                    try:
                        chunk = conn.recv(4096)
                    except socket.timeout:
                        continue
                    if not chunk:
                        break
                    buf += chunk
                    continue
                op, payload = frame
                if op == 0x8:
                    with send_lock:
                        conn.sendall(ws_frame(0x8, b""))
                    break
                if op == 0x9:
                    with send_lock:
                        conn.sendall(ws_frame(0xA, payload))
                elif op == 0x1:
                    try:
                        msg = json.loads(payload.decode())
                        reply = self.bcd.handle_message(msg) if isinstance(msg, dict) else None
                    except (ValueError, UnicodeDecodeError):
                        reply = {"t": "error", "msg": "bad json"}
                    except Exception as exc:       # noqa: BLE001
                        reply = {"t": "error", "msg": str(exc)}
                    if reply is not None:
                        send_text(json.dumps(reply))
        finally:
            if send_text in self.bcd.listeners:
                self.bcd.listeners.remove(send_text)
            # client gone: the dead-man timer zeroes the target within 300 ms


def ws_frame(opcode, payload):
    n = len(payload)
    if n < 126:
        head = struct.pack("!BB", 0x80 | opcode, n)
    elif n < 65536:
        head = struct.pack("!BBH", 0x80 | opcode, 126, n)
    else:
        head = struct.pack("!BBQ", 0x80 | opcode, 127, n)
    return head + payload


def ws_parse(buf):
    """Pop one complete frame from the bytearray; None if incomplete."""
    if len(buf) < 2:
        return None
    b0, b1 = buf[0], buf[1]
    opcode = b0 & 0x0F
    masked = b1 & 0x80
    n = b1 & 0x7F
    pos = 2
    if n == 126:
        if len(buf) < 4:
            return None
        n = struct.unpack_from("!H", buf, 2)[0]
        pos = 4
    elif n == 127:
        if len(buf) < 10:
            return None
        n = struct.unpack_from("!Q", buf, 2)[0]
        pos = 10
    if masked:
        if len(buf) < pos + 4 + n:
            return None
        mask = bytes(buf[pos:pos + 4])
        pos += 4
        payload = bytearray(buf[pos:pos + n])
        for i in range(n):
            payload[i] ^= mask[i & 3]
    else:
        if len(buf) < pos + n:
            return None
        payload = bytearray(buf[pos:pos + n])
    del buf[:pos + n]
    return opcode, bytes(payload)


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="balance control daemon")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--params", default="/mnt/data/bc_params.json")
    ap.add_argument("--shm-file", help="use a file instead of /dev/mem (development)")
    ap.add_argument("--no-mailbox", action="store_true")
    ap.add_argument("--no-emergency", action="store_true")
    ap.add_argument("--web", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "web"))
    args = ap.parse_args()

    def log(msg):
        print("bcd: " + msg, flush=True)

    shm = S.Shm.open_file(args.shm_file) if args.shm_file else S.Shm.open_devmem()
    t0 = time.time()
    while not shm.ready():
        if time.time() - t0 > 60:
            log("RTOS shared memory not ready after 60 s; continuing, will keep polling")
            break
        time.sleep(0.2)
    mbox = S.RecordingMailbox() if args.no_mailbox else S.Mailbox()
    emergency = None
    if not args.no_emergency and not args.shm_file:
        try:
            emergency = DevMemGpio().clear_stby
        except OSError as exc:
            log("emergency STBY path unavailable: %s" % exc)
    bcd = Bcd(shm, mbox, args.params, emergency, log)
    bcd.run_forever()
    if shm.ready():
        bcd.load_params()
    server = WsServer(bcd, port=args.port, static_dir=args.web)
    log("RTOS build %s boot #%d; listening on :%d" % (
        shm.header()["build_id"] if shm.ready() else "?", shm.header()["boot_count"], server.port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    bcd.stop()


if __name__ == "__main__":
    main()
