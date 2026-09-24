# burnd (Arduino upload daemon)

Source: https://github.com/kubuds/sophgo-arduino (tools/burn), branch sg200x-dev.

Builds a static daemon that receives Arduino IDE firmware over USB ACM and
loads it onto the C906L little core via Linux remoteproc sysfs.

```bash
# RISC-V (default)
make
# AArch64 (glibc arm64 big-core images)
make ARCH=aarch64
```

Prebuilt binaries are installed into:
- `device/generic/br_overlay/musl_riscv64/usr/bin/burnd`
- `device/generic/br_overlay/64bit/usr/bin/burnd`  (SDK_VER for TOOLCHAIN_GLIBC_ARM64)
