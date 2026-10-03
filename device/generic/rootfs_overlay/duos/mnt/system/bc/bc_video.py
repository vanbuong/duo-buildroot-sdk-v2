"""Video profiles and adaptive-rate decision for the balance-car stream.

Pure logic (tested on the host). Whether a profile actually changes the picture
depends on the RTSP/media binary that bc-video.sh starts reading
/mnt/data/bc_video.json at launch; this module only decides and records.
"""
import json
import os

# name: (width, height, fps, bitrate_kbps, codec)
PROFILES = {
    "fpv-low": (640, 360, 15, 800, "h264"),
    "fpv-med": (1280, 720, 20, 2000, "h264"),
    "fpv-hq": (1920, 1080, 25, 4000, "h264"),
    "mjpeg": (640, 480, 15, 4000, "mjpeg"),
}
ORDER = ("fpv-low", "fpv-med", "fpv-hq")
DEFAULT = "fpv-med"
CONFIG_PATH = "/mnt/data/bc_video.json"


def profile_dict(name):
    w, h, fps, kbps, codec = PROFILES[name]
    return {"profile": name, "width": w, "height": h, "fps": fps, "bitrate_kbps": kbps, "codec": codec}


def save(name, path=CONFIG_PATH):
    d = profile_dict(name)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(d, f)
    os.replace(tmp, path)
    return d


def load(path=CONFIG_PATH):
    try:
        with open(path) as f:
            d = json.load(f)
        if d.get("profile") in PROFILES:
            return d["profile"]
    except (OSError, ValueError):
        pass
    return DEFAULT


def env_lines(name):
    d = profile_dict(name)
    return ["BC_VIDEO_PROFILE=%s" % d["profile"], "BC_VIDEO_WIDTH=%d" % d["width"],
            "BC_VIDEO_HEIGHT=%d" % d["height"], "BC_VIDEO_FPS=%d" % d["fps"],
            "BC_VIDEO_KBPS=%d" % d["bitrate_kbps"], "BC_VIDEO_CODEC=%s" % d["codec"]]


class AdaptiveRate:
    """Steps the profile down fast when the Wi-Fi link is bad, up slowly when good.

    Input: RSSI in dBm (None = unknown) and a lost-telemetry counter. Hysteresis:
    down after `down_after` consecutive bad samples, up after `up_after` good ones.
    Only the fpv-* ladder is adapted; "mjpeg" or a manual choice is left alone.
    """

    def __init__(self, profile=DEFAULT, bad_dbm=-75, good_dbm=-62, down_after=2, up_after=10):
        self.profile = profile
        self.bad_dbm, self.good_dbm = bad_dbm, good_dbm
        self.down_after, self.up_after = down_after, up_after
        self.bad = self.good = 0
        self.last_lost = None
        self.auto = True

    def set_manual(self, profile):
        if profile not in PROFILES:
            raise ValueError("unknown profile %r" % profile)
        self.profile = profile
        self.auto = False
        self.bad = self.good = 0

    def set_auto(self):
        self.auto = True

    def update(self, rssi, lost=0):
        """Returns the new profile name if it changed, else None."""
        if not self.auto or self.profile not in ORDER:
            return None
        lost_inc = 0 if self.last_lost is None else max(0, lost - self.last_lost)
        self.last_lost = lost
        bad = (rssi is not None and rssi <= self.bad_dbm) or lost_inc > 20
        good = (rssi is not None and rssi >= self.good_dbm) and lost_inc == 0
        self.bad = self.bad + 1 if bad else 0
        self.good = self.good + 1 if good else 0
        i = ORDER.index(self.profile)
        if self.bad >= self.down_after and i > 0:
            self.profile, self.bad, self.good = ORDER[i - 1], 0, 0
            return self.profile
        if self.good >= self.up_after and i < len(ORDER) - 1:
            self.profile, self.bad, self.good = ORDER[i + 1], 0, 0
            return self.profile
        return None


if __name__ == "__main__":
    import sys
    if "--env" in sys.argv:
        print("\n".join("export " + line for line in env_lines(load())))
