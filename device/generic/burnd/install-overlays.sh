#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

make clean || true
make
install -D -m 755 burnd "$ROOT/../br_overlay/musl_riscv64/usr/bin/burnd"

make clean || true
make ARCH=aarch64
install -D -m 755 burnd "$ROOT/../br_overlay/64bit/usr/bin/burnd"

file "$ROOT/../br_overlay/musl_riscv64/usr/bin/burnd"
file "$ROOT/../br_overlay/64bit/usr/bin/burnd"
