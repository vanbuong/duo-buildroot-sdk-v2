#!/bin/bash
# Compile every DuoS device tree (arm64/riscv, sd/emmc) with cpp + dtc.
# Needs: device-tree-compiler python3 cpp
set -euo pipefail
REPO=$(cd "$(dirname "$0")/../../../../.." && pwd)
WORK=$(mktemp -d)
fail=0
for board in "$REPO"/build/boards/cv181x/*duos*; do
    name=$(basename "$board")
    dts_dir=$(ls -d "$board"/dts_*)
    arch=$([ "${dts_dir##*dts_}" = arm64 ] && echo arm || echo riscv)
    base=$(dirname "$(find "$REPO/build/boards/default/dts" -name "cv181x_base_${arch}.dtsi" | head -1)")
    mkdir -p "$WORK/$name"
    for t in h; do
        python3 "$REPO/build/scripts/mmap_conv.py" --type $t "$board/memmap.py" "$WORK/$name/cvi_board_memmap.h" >/dev/null
    done
    if cpp -nostdinc -undef -x assembler-with-cpp -I "$base" -I "$REPO/build/boards/default/dts/cv181x" \
        -I "$dts_dir" -I "$REPO/linux_5.10/include" -I "$WORK/$name" -include "$WORK/$name/cvi_board_memmap.h" \
        "$dts_dir"/*.dts 2>"$WORK/cpp.err" | dtc -I dts -O dtb -o "$WORK/$name/out.dtb" - 2>"$WORK/dtc.err"; then
        echo "OK   $name"
    else
        echo "FAIL $name"; cat "$WORK/cpp.err" "$WORK/dtc.err" | head -20; fail=1
    fi
done
exit $fail
