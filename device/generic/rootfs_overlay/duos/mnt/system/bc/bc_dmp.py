"""Experimental DMP (MotionApps 6.12) image handling for est_mode 2.

The image is NOT shipped in this repository. Pack it once from the ElectronicCats
MPU6050 library (MIT wrapper around InvenSense's image; check the licence of the
image yourself before redistributing it):

    bcctl dmp-pack MPU6050_6Axis_MotionApps612.cpp /mnt/data/bc_dmp612.bin

bcd then copies /mnt/data/bc_dmp612.bin into the shared window at start-up (and
whenever the RTOS restarts); the RTOS loads it when est_mode is set to 2.
"""
import re
import struct

import bc_layout as L
import bc_shm as S

IMAGE_SIZE = 3062
# crc32 of the MotionApps 6.12 image as published in ElectronicCats/mpu6050
# (src/MPU6050_6Axis_MotionApps612.cpp, dmpMemory[3062]); guards against an edited
# or truncated image, which must never be written into the sensor.
IMAGE_CRC32 = 0x386B78EE
DEFAULT_PATH = "/mnt/data/bc_dmp612.bin"


class DmpError(Exception):
    pass


def extract_from_cpp(text):
    """Return the dmpMemory[] bytes from MPU6050_6Axis_MotionApps612.cpp source text."""
    m = re.search(r"dmpMemory\s*\[\s*MPU6050_DMP_CODE_SIZE\s*\][^=]*=\s*\{", text)
    if not m:
        raise DmpError("dmpMemory[] not found - is this MPU6050_6Axis_MotionApps612.cpp?")
    end = text.index("};", m.end())
    body = text[m.end():end]
    body = re.sub(r"/\*.*?\*/", "", body, flags=re.S)       # block comments
    body = re.sub(r"//[^\n]*", "", body)                    # line comments (they contain hex numbers!)
    data = bytes(int(t, 16) for t in re.findall(r"0[xX]([0-9A-Fa-f]{2})\b", body))
    return data


def validate(data, force=False):
    if len(data) != IMAGE_SIZE:
        raise DmpError("image is %d bytes, expected %d" % (len(data), IMAGE_SIZE))
    crc = S.crc32(data)
    if crc != IMAGE_CRC32 and not force:
        raise DmpError("image crc32 0x%08X differs from the known MotionApps 6.12 image 0x%08X "
                       "(use --force only if you know why)" % (crc, IMAGE_CRC32))
    return crc


def pack(src_cpp, dst, force=False):
    with open(src_cpp, encoding="utf-8", errors="replace") as f:
        data = extract_from_cpp(f.read())
    crc = validate(data, force)
    with open(dst, "wb") as f:
        f.write(data)
    return len(data), crc


def load_file(path, force=False):
    with open(path, "rb") as f:
        data = f.read()
    validate(data, force)
    return data


def write_image(shm, data, kind=L.DMP_KIND_612):
    """Seqlock write of the image into the shared window (Linux is the only writer)."""
    if not data or len(data) > L.DMP_DATA_MAX:
        raise DmpError("image size %d out of range" % len(data))
    m = shm.m
    base = L.OFF_DMP
    seq = shm.u32(base) | 1
    shm.set_u32(base, seq)                               # odd: in progress
    hdr = struct.pack("<II", kind, len(data))
    m[base + 8:base + 16] = hdr
    m[base + 16:base + 16 + len(data)] = data
    crc = S.crc32(hdr + bytes(data))
    shm.set_u32(base + 4, crc)
    shm.set_u32(base, seq + 1)
    return seq + 1


def read_image(shm):
    """(kind, data) of a valid image block, or None."""
    base = L.OFF_DMP
    s1 = shm.u32(base)
    if s1 == 0 or s1 & 1:
        return None
    crc, kind, size = struct.unpack_from("<III", shm.m, base + 4)
    if size == 0 or size > L.DMP_DATA_MAX:
        return None
    data = bytes(shm.m[base + 16:base + 16 + size])
    if shm.u32(base) != s1 or crc != S.crc32(struct.pack("<II", kind, size) + data):
        return None
    return kind, data


def state_name(status):
    st = status["dmp_info"] & 0xFF
    return L.DMP_STATES[st] if st < len(L.DMP_STATES) else "?"


def fallbacks(status):
    return status["dmp_info"] >> 8
