#!/bin/sh
# Camera selection + RTSP server. Failure here must never affect bcd/RTOS.
BC=/mnt/system/bc
export LD_LIBRARY_PATH=/mnt/system/lib:/mnt/system/usr/lib:/mnt/system/usr/lib/3rd:$LD_LIBRARY_PATH

python3 $BC/bc_camera.py || { echo "bc-video: no usable camera"; exit 0; }

# Stream profile chosen in the web UI / adaptive-rate logic (bc_video.py). The
# values are exported for the RTSP binary; whether it honours them depends on
# that binary (not verified on hardware).
eval "$(python3 $BC/bc_video.py --env 2>/dev/null)"
echo "bc-video: profile=${BC_VIDEO_PROFILE:-default} ${BC_VIDEO_WIDTH:-?}x${BC_VIDEO_HEIGHT:-?}@${BC_VIDEO_FPS:-?} ${BC_VIDEO_KBPS:-?}kbps"
export BC_VIDEO_PROFILE BC_VIDEO_WIDTH BC_VIDEO_HEIGHT BC_VIDEO_FPS BC_VIDEO_KBPS BC_VIDEO_CODEC

for exe in /mnt/system/usr/example/rtsp_server_video /mnt/system/usr/bin/rtsp_server_video \
           /mnt/system/usr/bin/cvi_rtsp_service; do
    if [ -x "$exe" ]; then
        echo "bc-video: starting $exe"
        exec "$exe"
    fi
done
echo "bc-video: no RTSP server binary found (build cvi_rtsp examples)"
