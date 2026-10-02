import os
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[5]
BC_DIR = REPO / "device/generic/rootfs_overlay/duos/mnt/system/bc"
sys.path.insert(0, str(BC_DIR))

TOOLS = Path(os.environ.get("BC_TOOLS", Path(__file__).resolve().parent.parent / "test/build"))


def tool(name, *args):
    exe = TOOLS / name
    if not exe.exists():
        raise unittest_skip(f"{exe} not built (make -C test tools)")
    return subprocess.run([str(exe), *map(str, args)], check=True, capture_output=True, text=True).stdout


def unittest_skip(msg):
    import unittest
    return unittest.SkipTest(msg)
