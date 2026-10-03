"""Size-rotating JSON-lines telemetry logger for bcd (flash-friendly: bounded size)."""
import json
import os


def _encode(o):
    if isinstance(o, (bytes, bytearray)):		# telemetry padding
        return bytes(o).hex()
    raise TypeError("not serialisable: %r" % type(o))


class TelemLogger:
    def __init__(self, directory, max_bytes=2 * 1024 * 1024, keep=3, prefix="telem"):
        self.dir = directory
        self.max_bytes = max_bytes
        self.keep = max(1, keep)
        self.prefix = prefix
        self.path = os.path.join(directory, prefix + ".jsonl")
        self.f = None
        self.size = 0
        self.errors = 0
        os.makedirs(directory, exist_ok=True)
        self._open()

    def _open(self):
        self.f = open(self.path, "a", buffering=1)
        self.size = os.path.getsize(self.path)

    def _rotate(self):
        """live -> .1 -> .2 ... ; `keep` rotated files are retained."""
        self.f.close()
        oldest = "%s.%d" % (self.path, self.keep)
        if os.path.exists(oldest):
            os.remove(oldest)
        for i in range(self.keep - 1, 0, -1):
            src = "%s.%d" % (self.path, i)
            if os.path.exists(src):
                os.replace(src, "%s.%d" % (self.path, i + 1))
        os.replace(self.path, self.path + ".1")
        self._open()

    def write(self, record):
        """record: dict (JSON serialisable). Never raises: logging must not hurt control."""
        try:
            line = json.dumps(record, separators=(",", ":"), default=_encode) + "\n"
            if self.size + len(line) > self.max_bytes:
                self._rotate()
            self.f.write(line)
            self.size += len(line)
        except (OSError, ValueError, TypeError):
            self.errors += 1

    def close(self):
        try:
            if self.f:
                self.f.close()
        except OSError:
            pass
