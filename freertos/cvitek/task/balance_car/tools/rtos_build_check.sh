#!/bin/bash
# Build the FreeRTOS image (cvirtos.elf) with a stock Ubuntu RISC-V GCC and fail on
# any warning in task/balance_car. The SDK normally uses the T-Head toolchain from
# milkv-duo/host-tools; this script adapts a scratch COPY of freertos/ so nothing
# in the repo is touched:
#   - T-Head custom CSR names (mhcr, mcor, ...) -> numeric addresses
#   - picolibc headers/libs instead of newlib; --specs=nosys.specs stub
# Usage: rtos_build_check.sh [board-dir-name] [chip]
#   defaults: sg2000_milkv_duos_glibc_arm64_sd  cv181x
# Needs: gcc-riscv64-unknown-elf picolibc-riscv64-unknown-elf cmake ninja-build python3
set -euo pipefail

BOARD=${1:-sg2000_milkv_duos_glibc_arm64_sd}
CHIP=${2:-cv181x}
REPO=$(cd "$(dirname "$0")/../../../../.." && pwd)
WORK=${WORK:-$(mktemp -d)}
PICO=/usr/lib/picolibc/riscv64-unknown-elf

BOARD_DIR=$(ls -d "$REPO"/build/boards/*/"$BOARD" | head -1)
FULLNAME=$BOARD
mkdir -p "$WORK/env/output/$FULLNAME" "$WORK/bin" "$WORK/compat/sys" "$WORK/stub"

# memory map headers/linker include, generated like the SDK's mmap.mk does
for t in h conf ld; do
    python3 "$REPO/build/scripts/mmap_conv.py" --type $t "$BOARD_DIR/memmap.py" \
        "$WORK/env/output/$FULLNAME/cvi_board_memmap.$t" >/dev/null
done
cat > "$WORK/env/.config" <<CFG
CONFIG_ENABLE_FREERTOS=y
CONFIG_FAST_IMAGE_TYPE="0"
CONFIG_ENABLE_RTOS_DUMP_PRINT=y
CONFIG_BOARD="${CHIP}_asic"
CFG

# toolchain shims
: > "$WORK/nosys.specs"
printf '#ifndef COMPAT_SYS_TIME_H\n#define COMPAT_SYS_TIME_H\nstruct timeval { long tv_sec; long tv_usec; };\nint gettimeofday(struct timeval *tv, void *tz);\n#endif\n' \
    > "$WORK/compat/sys/time.h"
riscv64-unknown-elf-ar rcs "$WORK/stub/libsim.a"
for t in gcc g++; do
cat > "$WORK/bin/riscv64-unknown-elf-$t" <<SH
#!/bin/bash
args=(); extra=()
for a in "\$@"; do
  [ "\$a" = "--specs=nosys.specs" ] && a="--specs=$WORK/nosys.specs"
  case "\$a" in *.c) extra=(-include $WORK/compat/sys/time.h);; esac
  args+=("\$a")
done
exec /usr/bin/riscv64-unknown-elf-$t "\${args[@]}" "\${extra[@]}" \\
    -L$PICO/lib/rv64iafd/lp64d -L$WORK/stub -isystem $WORK/compat -isystem $PICO/include
SH
chmod +x "$WORK/bin/riscv64-unknown-elf-$t"
done
for t in objcopy objdump ar ranlib size nm ld as strip; do
    ln -sf "/usr/bin/riscv64-unknown-elf-$t" "$WORK/bin/riscv64-unknown-elf-$t"
done

# scratch copy of the RTOS tree
mkdir -p "$WORK/freertos"
cp -r "$REPO/freertos/cvitek" "$WORK/freertos/cvitek"
cp -r "$REPO/freertos/Source" "$WORK/freertos/Source"
cd "$WORK/freertos/cvitek"
rm -rf build install
for f in $(grep -rlE 'csr[rws]' arch kernel common hal driver task 2>/dev/null); do
    perl -pi -e 'if (/csr[rwsci]+/) { s/\bmxstatus\b/0x7c0/g; s/\bmhcr\b/0x7c1/g; s/\bmcor\b/0x7c2/g; s/\bmhint\b/0x7c5/g; }' "$f"
done

export BUILD_PATH=$WORK/env PROJECT_FULLNAME=$FULLNAME DDR_64MB_SIZE=n PATH=$WORK/bin:$PATH
"./build_${CHIP}.sh" > "$WORK/build.log" 2>&1 || { tail -40 "$WORK/build.log"; echo "RTOS build FAILED"; exit 1; }

ELF=install/bin/cvirtos.elf
riscv64-unknown-elf-size "$ELF"
if grep -nE "balance_car/.*(warning|error):" "$WORK/build.log"; then
    echo "warnings in task/balance_car"; exit 1
fi
SYMS=$(riscv64-unknown-elf-nm "$ELF")
if [ "$CHIP" = cv181x ]; then
    grep -q " _bc_shm_base$" <<<"$SYMS" || { echo "_bc_shm_base missing"; exit 1; }
    grep -E " _bc_shm_base$| _end$" <<<"$SYMS"
    # firmware size budget: the image (text+data+bss up to _end) must leave at least
    # 256 KiB of the 2 MiB carve-out free below the shared window (heap/stack growth)
    END=$(awk '$3=="_end"{print $1}' <<<"$SYMS")
    BASE=$((0x9FE00000))
    USED=$(( 0x$END - BASE ))
    BUDGET=$(( 2*1024*1024 - 64*1024 - 256*1024 ))
    echo "firmware footprint: $USED bytes (budget $BUDGET)"
    [ "$USED" -le "$BUDGET" ] || { echo "firmware too large for budget"; exit 1; }
    grep -q " prvBalanceCommTask$" <<<"$SYMS" || { echo "balance comm task not linked"; exit 1; }
else
    if grep -q " balance_car_start$" <<<"$SYMS"; then
        echo "balance_car must not be linked for $CHIP"; exit 1
    fi
fi
echo "RTOS build OK ($BOARD, $CHIP): $WORK/freertos/cvitek/$ELF"
