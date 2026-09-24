#!/bin/sh
# Helper: load an ELF onto C906L via remoteproc (no hang if file missing).
set -e

RPROC=/sys/class/remoteproc/remoteproc0
FW_NAME="${1:-arduino.elf}"
FW_PATH="/lib/firmware/${FW_NAME}"

if [ ! -e "$RPROC/state" ]; then
  echo "remoteproc0 not present (is CONFIG_CVITEK_REMOTEPROC enabled?)" >&2
  exit 1
fi

if [ ! -f "$FW_PATH" ]; then
  echo "missing firmware: $FW_PATH" >&2
  echo "Upload a sketch from Arduino IDE (burnd), or copy an ELF there." >&2
  exit 1
fi

mkdir -p /lib/firmware
echo "$FW_NAME" > "$RPROC/firmware"

state=$(cat "$RPROC/state")
if [ "$state" = "running" ]; then
  echo stop > "$RPROC/state"
fi

echo start > "$RPROC/state"
echo "remoteproc state: $(cat "$RPROC/state") firmware: $(cat "$RPROC/firmware")"
