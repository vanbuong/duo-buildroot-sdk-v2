#!/usr/bin/env python3
"""bcctl - command line tool for the balance robot.

  bcctl status | watch [-n N] | events
  bcctl arm | disarm | estop | target V W | ping
  bcctl get [NAME] | set NAME VALUE | save | load
  bcctl calib gyro|trim
Works without bcd (talks to the shared window and the mailbox directly).
"""
import argparse
import json
import sys
import time

import bc_layout as L
import bc_shm as S

PARAMS_FILE = "/mnt/data/bc_params.json"


def open_all(args):
    shm = S.Shm.open_file(args.shm_file) if args.shm_file else S.Shm.open_devmem()
    mbox = S.RecordingMailbox() if args.no_mailbox else S.Mailbox()
    return shm, mbox


def fmt_tel(r):
    return ("%-9s %-10s pitch %7.2f acc %7.2f  v %6.3f/%6.3f  motor %6.1f/%6.1f  enc %d/%d  "
            "T=%dus exec=%dus" % (
                L.STATE_NAMES[r["state"]] if r["state"] < 8 else "?",
                L.FAULT_NAMES[r["fault"]] if r["fault"] < len(L.FAULT_NAMES) else "?",
                r["pitch_cdeg"] / 100, r["pitch_acc_cdeg"] / 100,
                r["speed_l_mmps"] / 1000, r["speed_r_mmps"] / 1000,
                r["motor_l_pm"] / 10, r["motor_r_pm"] / 10, r["enc_l"], r["enc_r"],
                r["period_us"], r["exec_us"]))


def apply_param(shm, mbox, changes, wait=1.0):
    echo = shm.read_echo()
    full = shm.current_params()
    full.update(changes)
    seq = shm.write_params(full)
    base = echo["applied_count"] if echo else 0
    mbox.send(L.BC_CMD_APPLY_PARAMS, seq)
    end = time.time() + wait
    while time.time() < end:
        e = shm.read_echo()
        if e and e["applied_seq"] == seq and e["applied_count"] > base:
            return True
        time.sleep(0.01)
    return False


def main(argv=None):
    ap = argparse.ArgumentParser(prog="bcctl")
    ap.add_argument("--shm-file")
    ap.add_argument("--no-mailbox", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    w = sub.add_parser("watch")
    w.add_argument("-n", type=int, default=0, help="stop after N records")
    sub.add_parser("events")
    for name in ("arm", "disarm", "estop", "ping", "save", "load"):
        sub.add_parser(name)
    t = sub.add_parser("target")
    t.add_argument("v", type=float)
    t.add_argument("w", type=float)
    g = sub.add_parser("get")
    g.add_argument("name", nargs="?")
    s = sub.add_parser("set")
    s.add_argument("name")
    s.add_argument("value", type=float)
    c = sub.add_parser("calib")
    c.add_argument("what", choices=("gyro", "trim"))
    args = ap.parse_args(argv)

    shm, mbox = open_all(args)
    if args.cmd == "status":
        print(json.dumps({"header": shm.header(), "status": shm.status()}, indent=1))
    elif args.cmd == "watch":
        rd = S.TelemReader(shm)
        n = 0
        while args.n == 0 or n < args.n:
            for r in rd.poll():
                print(fmt_tel(r))
                n += 1
                if args.n and n >= args.n:
                    break
            time.sleep(0.02)
    elif args.cmd == "events":
        rd = S.EventReader(shm, from_start=True)
        for ev in rd.poll(L.EVENT_SLOTS):
            print("%10d us type=%d arg=%d val=%d" % (ev["t_us"], ev["type"], ev["arg"], ev["val"]))
    elif args.cmd == "arm":
        mbox.send(L.BC_CMD_ARM, 0)
    elif args.cmd == "disarm":
        mbox.send(L.BC_CMD_DISARM, 0)
    elif args.cmd == "estop":
        mbox.send(L.BC_CMD_ESTOP, 0)
    elif args.cmd == "ping":
        mbox.send(L.BC_CMD_PING, int(time.time()) & 0xFFFFFFFF)
    elif args.cmd == "target":
        mbox.send(L.BC_CMD_SET_TARGET, S.pack_target(args.v * 1000, args.w * 1000))
    elif args.cmd == "get":
        echo = shm.read_echo()
        if not echo or echo["applied_count"] == 0:
            print("RTOS has not published parameters yet", file=sys.stderr)
            return 1
        p = echo["params"]
        if args.name:
            if args.name not in p:
                print("unknown parameter", file=sys.stderr)
                return 2
            print(p[args.name])
        else:
            for k in sorted(p):
                print("%-16s %s" % (k, p[k]))
    elif args.cmd == "set":
        if S.param_index(args.name) < 0:
            print("unknown parameter", file=sys.stderr)
            return 2
        v = args.value
        if L.PARAMS[S.param_index(args.name)][1] != "F":
            v = int(v)
        try:
            ok = apply_param(shm, mbox, {args.name: v})
        except S.ShmError as exc:
            print("rejected:", exc, file=sys.stderr)
            return 2
        print("applied" if ok else "RTOS did not confirm")
        return 0 if ok else 3
    elif args.cmd == "save":
        with open(PARAMS_FILE, "w") as f:
            json.dump(shm.current_params(), f, indent=1, sort_keys=True)
        print("saved", PARAMS_FILE)
    elif args.cmd == "load":
        with open(PARAMS_FILE) as f:
            ok = apply_param(shm, mbox, json.load(f))
        print("applied" if ok else "RTOS did not confirm")
    elif args.cmd == "calib":
        mbox.send(L.BC_CMD_CALIBRATE, L.BC_CALIB_GYRO if args.what == "gyro" else L.BC_CALIB_TRIM)
    return 0


if __name__ == "__main__":
    sys.exit(main())
