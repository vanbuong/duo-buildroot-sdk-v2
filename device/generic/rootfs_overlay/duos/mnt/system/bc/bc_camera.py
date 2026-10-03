#!/usr/bin/env python3
"""Select the active CSI camera on the DuoS (one at a time) - docs/balance_car/02.

  J1  16-pin 1.8 V connector (ships with GC2083): I2C3, lanes 2/0/1, reset XGPIOA_2
  J2  15-pin Raspberry-Pi connector (OV5647):     I2C2, lanes 5/3/4, reset XGPIOA_4

Probes the sensor chip ids (best effort: the sensor only answers when its MCLK
runs), writes the matching /mnt/data/sensor_cfg.ini and holds the other
connector's sensor in reset. Chip ids are from the sensor datasheets and must
be verified on hardware.
"""
import argparse
import fcntl
import json
import os
import shutil
import sys
import time

I2C_SLAVE = 0x0703
GPIO_A_BASE = 480          # Linux numbering of XGPIOA_n (verify on the image)

# connector -> dict(bus, reset_gpio, candidates: [(sensor, addr, reg, regbytes, expect, ini)])
CONNECTORS = {
    "j1": dict(bus=3, reset=GPIO_A_BASE + 2, sensors=[
        ("GC2083", 0x37, 0xF0, 1, bytes([0x20, 0x83]), "sensor_cfg_GC2083.ini"),
        ("OV5647", 0x36, 0x300A, 2, bytes([0x56, 0x47]), "sensor_cfg_OV5647_J1.ini"),
    ]),
    "j2": dict(bus=2, reset=GPIO_A_BASE + 4, sensors=[
        ("OV5647", 0x36, 0x300A, 2, bytes([0x56, 0x47]), "sensor_cfg_OV5647_J2.ini"),
        ("IMX219", 0x10, 0x0000, 2, bytes([0x02, 0x19]), None),     # no SDK driver yet
    ]),
}


def i2c_read(bus, addr, reg, regbytes, n=2, dev_fmt="/dev/i2c-%d"):
    fd = os.open(dev_fmt % bus, os.O_RDWR)
    try:
        fcntl.ioctl(fd, I2C_SLAVE, addr)
        os.write(fd, reg.to_bytes(regbytes, "big"))
        return os.read(fd, n)
    finally:
        os.close(fd)


class Gpio:
    def __init__(self, root="/sys/class/gpio"):
        self.root = root

    def set(self, num, value):
        d = os.path.join(self.root, "gpio%d" % num)
        if not os.path.isdir(d):
            with open(os.path.join(self.root, "export"), "w") as f:
                f.write(str(num))
        with open(os.path.join(d, "direction"), "w") as f:
            f.write("out")
        with open(os.path.join(d, "value"), "w") as f:
            f.write("1" if value else "0")


def probe_connector(name, reader=i2c_read, gpio=None, settle=0.1):
    """Return (sensor, ini) found on a connector, or None."""
    c = CONNECTORS[name]
    if gpio:
        gpio.set(c["reset"], 1)            # release reset (active low)
        time.sleep(settle)
    for sensor, addr, reg, rb, expect, ini in c["sensors"]:
        try:
            if reader(c["bus"], addr, reg, rb, len(expect)) == expect:
                return sensor, ini
        except OSError:
            continue
    return None


def choose(found, preferred="j1", forced=None):
    """found: {'j1': (sensor, ini)|None, 'j2': ...}. Returns (connector, sensor, ini, why)."""
    if forced in CONNECTORS:
        s = found.get(forced)
        if s:
            return forced, s[0], s[1], "forced"
        default = CONNECTORS[forced]["sensors"][0]
        return forced, default[0], default[5], "forced (not probed)"
    present = [k for k in ("j1", "j2") if found.get(k)]
    if len(present) == 1:
        k = present[0]
        return k, found[k][0], found[k][1], "only one detected"
    if len(present) == 2:
        k = preferred if preferred in present else present[0]
        return k, found[k][0], found[k][1], "both detected, preferred"
    default = CONNECTORS[preferred]["sensors"][0]
    return preferred, default[0], default[5], "none detected (MCLK off?), using preferred"


def read_conf(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def select(args, reader=i2c_read, gpio=None):
    conf = read_conf(args.conf)
    forced = args.force if args.force != "auto" else conf.get("force")
    preferred = conf.get("preferred", args.prefer)
    found = {}
    if forced not in CONNECTORS:
        for k in CONNECTORS:
            found[k] = probe_connector(k, reader, gpio)
    conn, sensor, ini, why = choose(found, preferred, forced if forced in CONNECTORS else None)
    result = {"connector": conn, "sensor": sensor, "ini": ini, "why": why,
              "detected": {k: (v[0] if v else None) for k, v in found.items()}}
    if ini is None:
        result["error"] = "%s has no SDK driver / ini" % sensor
        return result
    src = os.path.join(args.data, ini)
    dst = os.path.join(args.data, "sensor_cfg.ini")
    if not os.path.exists(src):
        result["error"] = "missing " + src
        return result
    if not args.dry_run:
        shutil.copyfile(src, dst)
        if gpio:
            for k, c in CONNECTORS.items():
                gpio.set(c["reset"], 1 if k == conn else 0)    # other sensor stays in reset
    return result


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--force", choices=("auto", "j1", "j2"), default="auto")
    ap.add_argument("--prefer", choices=("j1", "j2"), default="j1")
    ap.add_argument("--conf", default="/mnt/data/bc_camera.conf")
    ap.add_argument("--data", default="/mnt/data")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--no-gpio", action="store_true")
    args = ap.parse_args(argv)
    res = select(args, gpio=None if (args.no_gpio or args.dry_run) else Gpio())
    print(json.dumps(res))
    return 2 if "error" in res else 0


if __name__ == "__main__":
    sys.exit(main())
