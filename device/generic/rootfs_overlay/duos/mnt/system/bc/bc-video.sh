#!/bin/sh
# Camera selection + RTSP server. Failure here must never affect bcd/RTOS.
BC=/mnt/system/bc
export LD_LIBRARY_PATH=/mnt/system/lib:/mnt/system/usr/lib:/mnt/system/usr/lib/3rd:$LD_LIBRARY_PATH

python3 $BC/bc_camera.py || { echo "bc-video: no usable camera"; exit 0; }

for exe in /mnt/system/usr/example/rtsp_server_video /mnt/system/usr/bin/rtsp_server_video \
           /mnt/system/usr/bin/cvi_rtsp_service; do
    if [ -x "$exe" ]; then
        echo "bc-video: starting $exe"
        exec "$exe"
    fi
done
echo "bc-video: no RTSP server binary found (build cvi_rtsp examples)"
