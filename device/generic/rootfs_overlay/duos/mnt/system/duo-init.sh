#!/bin/sh

function set_gpio()
{
	local gpio_num=$1
	local gpio_val=$2
	local gpio_path="/sys/class/gpio/gpio${gpio_num}"

	if test -d ${gpio_path}; then
		echo "GPIO ${gpio_num} already exported" >> /tmp/gpio.log 2>&1
	else
		echo ${gpio_num} > /sys/class/gpio/export
	fi

	echo out > ${gpio_path}/direction
	sleep 0.1
	echo ${gpio_val} > ${gpio_path}/value
}

# Hardware V1.1
gpio_b17=465
set_gpio ${gpio_b17} 0

# Host Wake BT
host_wake_bt=362
set_gpio ${host_wake_bt} 1

# WIFI/BT Module
insmod /mnt/system/ko/aic8800_bsp.ko
sleep 0.5
insmod /mnt/system/ko/aic8800_fdrv.ko

# The PWM0 block (VIVO_D9/D10) belongs to the balance-robot firmware on the C906L
# (docs/balance_car/01), so the Linux PWM module is not loaded here.
# insmod /mnt/system/ko/cv181x_pwm.ko

# Balance robot services: bcd (teleop/telemetry), Wi-Fi, camera + RTSP.
if [ -f /mnt/system/bc/bc-start.sh ]; then
    sh /mnt/system/bc/bc-start.sh &
fi

