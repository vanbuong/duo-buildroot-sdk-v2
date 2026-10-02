"""Access to the RTOS<->Linux shared-memory window and the cmdqu mailbox.

Mirrors freertos/cvitek/task/balance_car/include/bc_shm.h (single writer per
cache line, seqlock records). Pure Python, no third-party modules.
"""
import fcntl
import mmap
import os
import struct
import time
import zlib

import bc_layout as L

# _IOW('r', CMDQU_SEND = 1, unsigned long) on 64-bit Linux
RTOS_CMDQU_SEND = (1 << 30) | (8 << 16) | (ord("r") << 8) | 1


def crc32(data):
    return zlib.crc32(data) & 0xFFFFFFFF


class ShmError(Exception):
    pass


class Shm:
    """View over the 64 KiB window (mmap object or any bytearray-like)."""

    def __init__(self, buf):
        self.m = buf

    # -- construction ---------------------------------------------------
    @classmethod
    def open_devmem(cls, phys=L.SHM_PHYS, path="/dev/mem"):
        fd = os.open(path, os.O_RDWR | os.O_SYNC)
        m = mmap.mmap(fd, L.SHM_SIZE, mmap.MAP_SHARED,
                      mmap.PROT_READ | mmap.PROT_WRITE, offset=phys)
        os.close(fd)
        return cls(m)

    @classmethod
    def open_file(cls, path, create=False):
        fd = os.open(path, os.O_RDWR | (os.O_CREAT if create else 0), 0o644)
        if create:
            os.ftruncate(fd, L.SHM_SIZE)
        m = mmap.mmap(fd, L.SHM_SIZE, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
        os.close(fd)
        return cls(m)

    @classmethod
    def anonymous(cls):
        return cls(mmap.mmap(-1, L.SHM_SIZE))

    # -- raw access -----------------------------------------------------
    def u32(self, off):
        return struct.unpack_from("<I", self.m, off)[0]

    def set_u32(self, off, val):
        struct.pack_into("<I", self.m, off, val & 0xFFFFFFFF)

    # -- header / status --------------------------------------------------
    def header(self):
        v = struct.unpack_from(L.HDR_FMT, self.m, L.OFF_HDR)
        return {"magic": v[0], "version": v[1], "size": v[2], "boot_count": v[3],
                "build_id": v[4].split(b"\0", 1)[0].decode("ascii", "replace")}

    def ready(self):
        h = self.header()
        return (h["magic"] == L.SHM_MAGIC and h["version"] == L.SHM_VERSION
                and h["size"] == L.SHM_SIZE)

    def status(self):
        v = struct.unpack_from(L.STATUS_FMT, self.m, L.OFF_STATUS)
        d = dict(zip(L.STATUS_FIELDS, v))
        d["state_name"] = (L.STATE_NAMES[d["state"]] if d["state"] < len(L.STATE_NAMES) else "?")
        d["fault_name"] = (L.FAULT_NAMES[d["fault"]] if d["fault"] < len(L.FAULT_NAMES) else "?")
        return d

    def bump_heartbeat(self):
        hb = (self.u32(L.OFF_LINUX) + 1) & 0xFFFFFFFF
        self.set_u32(L.OFF_LINUX, hb)
        return hb

    # -- seqlock rings (RTOS writes, Linux reads) --------------------------
    def _read_ring(self, base, slots, size, fmt, fields, head_field, n, crc_span):
        head = self.status()[head_field]
        if n >= head:
            return 1, None
        if head - n > slots:
            return -1, None
        off = base + (n % slots) * size
        want = 2 * n + 2
        s1 = self.u32(off)
        if s1 > want:
            return -1, None
        if s1 != want:
            return -2, None
        raw = bytes(self.m[off:off + size])
        s2 = self.u32(off)
        if s2 != want:
            return (-1 if s2 > want else -2), None
        if crc_span and struct.unpack_from("<I", raw, size - 4)[0] != crc32(raw[4:4 + crc_span]):
            return -2, None
        return 0, dict(zip(fields, struct.unpack(fmt, raw)))

    def read_telem(self, n):
        return self._read_ring(L.OFF_TELEM, L.TELEM_SLOTS, 64, L.TELEM_FMT,
                               L.TELEM_FIELDS, "telem_head", n, 56)

    def read_event(self, n):
        return self._read_ring(L.OFF_EVENTS, L.EVENT_SLOTS, 32, L.EVENT_FMT,
                               L.EVENT_FIELDS, "event_head", n, 0)

    # -- parameters ------------------------------------------------------------
    def read_echo(self):
        """Parameters the RTOS is actually using. Returns dict or None if torn."""
        sz = struct.calcsize(L.ECHO_FMT)
        for _ in range(10):
            s1 = self.u32(L.OFF_ECHO)
            if s1 & 1:
                continue
            raw = bytes(self.m[L.OFF_ECHO:L.OFF_ECHO + sz])
            if self.u32(L.OFF_ECHO) != s1:
                continue
            v = struct.unpack(L.ECHO_FMT, raw)
            return {"seq": v[0], "applied_count": v[1], "last_result": v[2],
                    "applied_seq": v[3], "params": values_to_dict(v[4:])}
        return None

    def current_params(self):
        """Parameters in use by the RTOS; defaults if it has not published a
        valid echo yet (all-zero block before the first apply)."""
        e = self.read_echo()
        if e and e["applied_count"] > 0 and validate_params(e["params"]) is None:
            return dict(e["params"])
        return defaults()

    def write_params(self, params):
        """Write a complete parameter set (dict name->value) into the seqlock
        block. Returns the new even sequence number."""
        full = fill_defaults(params)
        err = validate_params(full)
        if err:
            raise ShmError(err)
        body = struct.pack("<II", L.SHM_VERSION, struct.calcsize(L.PARAM_FMT)) + \
            struct.pack(L.PARAM_FMT, *dict_to_values(full))
        blk = struct.pack("<I", crc32(body)) + body          # crc, version, size, params
        seq = self.u32(L.OFF_PARAM) | 1
        self.set_u32(L.OFF_PARAM, seq)                       # odd: in progress
        self.m[L.OFF_PARAM + 4:L.OFF_PARAM + 4 + len(blk)] = blk
        self.set_u32(L.OFF_PARAM, seq + 1)
        return seq + 1


class TelemReader:
    """Follows the telemetry ring, counting records lost to overrun."""

    def __init__(self, shm):
        self.shm = shm
        self.next = shm.status()["telem_head"]
        self.lost = 0
        self.torn = 0

    def poll(self, limit=64):
        out = []
        while len(out) < limit:
            head = self.shm.status()["telem_head"]
            if self.next >= head:
                break
            if head - self.next > L.TELEM_SLOTS - 8:
                skip = head - (L.TELEM_SLOTS - 8) - self.next
                self.lost += skip
                self.next += skip
            rc, rec = self.shm.read_telem(self.next)
            if rc == 0:
                out.append(rec)
                self.next += 1
            elif rc == -1:
                self.lost += 1
                self.next += 1
            elif rc == -2:
                self.torn += 1
                break                      # writer mid-update; retry next poll
            else:
                break
        return out


class EventReader:
    def __init__(self, shm, from_start=False):
        self.shm = shm
        self.next = 0 if from_start else shm.status()["event_head"]

    def poll(self, limit=64):
        out = []
        while len(out) < limit:
            head = self.shm.status()["event_head"]
            if self.next >= head:
                break
            if head - self.next > L.EVENT_SLOTS - 4:
                self.next = head - (L.EVENT_SLOTS - 4)
            rc, rec = self.shm.read_event(self.next)
            if rc == 0:
                out.append(rec)
                self.next += 1
            elif rc == -1:
                self.next += 1
            else:
                break
        return out


# ---- parameter helpers ------------------------------------------------------
def param_index(name):
    for i, p in enumerate(L.PARAMS):
        if p[0] == name:
            return i
    return -1


def defaults():
    return {p[0]: p[2] for p in L.PARAMS}


def fill_defaults(d):
    full = defaults()
    unknown = set(d) - set(full)
    if unknown:
        raise ShmError("unknown parameter(s): " + ", ".join(sorted(unknown)))
    full.update(d)
    return full


def validate_params(d):
    """Return an error string, or None. Same rules as bc_params_validate()."""
    for name, typ, _dflt, lo, hi in L.PARAMS:
        v = d.get(name)
        if v is None:
            return f"{name}: missing"
        try:
            fv = float(v)
        except (TypeError, ValueError):
            return f"{name}: not a number"
        if fv != fv or fv in (float("inf"), float("-inf")):
            return f"{name}: not finite"
        if fv < lo or fv > hi:
            return f"{name}: {v} outside [{lo}, {hi}]"
        if typ == "S" and fv not in (1.0, -1.0):
            return f"{name}: must be +1 or -1"
        if typ != "F" and fv != int(fv):
            return f"{name}: must be an integer"
    return None


def dict_to_values(d):
    return [float(d[p[0]]) if p[1] == "F" else int(d[p[0]]) for p in L.PARAMS]


def values_to_dict(vals):
    out = {}
    for p, v in zip(L.PARAMS, vals):
        out[p[0]] = float(v) if p[1] == "F" else int(v)
    return out


# ---- mailbox ---------------------------------------------------------------
class Mailbox:
    """Sends 8-byte cmdqu_t messages to the RTOS through /dev/cvi-rtos-cmdqu."""

    def __init__(self, dev="/dev/cvi-rtos-cmdqu"):
        self.fd = os.open(dev, os.O_RDWR)

    @staticmethod
    def pack(cmd_id, param, block=0):
        return struct.pack("<BBHI", L.BC_IP_ID, (cmd_id & 0x7F) | ((block & 1) << 7),
                           0, param & 0xFFFFFFFF)

    def send(self, cmd_id, param=0):
        buf = bytearray(self.pack(cmd_id, param))
        fcntl.ioctl(self.fd, RTOS_CMDQU_SEND, buf)

    def close(self):
        os.close(self.fd)


class RecordingMailbox:
    """Test double: remembers what would have been sent."""

    def __init__(self):
        self.sent = []

    def send(self, cmd_id, param=0):
        self.sent.append((cmd_id, param & 0xFFFFFFFF))

    def close(self):
        pass


def pack_target(speed_mmps, turn_mradps):
    s = max(-32768, min(32767, int(speed_mmps)))
    t = max(-32768, min(32767, int(turn_mradps)))
    return ((t & 0xFFFF) << 16) | (s & 0xFFFF)


def now():
    return time.monotonic()
