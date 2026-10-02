#!/bin/sh
# Start the balance robot services. Order matters: RTOS + bcd first, video last,
# so the robot stays controllable even if the camera pipeline fails.
BC=/mnt/system/bc
LOG=/tmp/bc.log

# Wait (max 30 s) for the RTOS image and the mailbox driver.
i=0
while [ ! -e /dev/cvi-rtos-cmdqu ] && [ $i -lt 30 ]; do sleep 1; i=$((i+1)); done

python3 $BC/bcd.py >>$LOG 2>&1 &
sh $BC/bc-net.sh >>$LOG 2>&1 &
sleep 2
sh $BC/bc-video.sh >>$LOG 2>&1 &
