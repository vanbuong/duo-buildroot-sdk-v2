#!/bin/sh
# Wi-Fi bring-up: STA first (wpa_supplicant), fall back to an own AP (hostapd,
# if present). Power-save off for latency. Configuration: /mnt/data/bc_wifi.conf
#   SSID=..  PSK=..  AP_SSID=balancebot  AP_PSK=balancebot  AP_ADDR=192.168.50.1
IF=wlan0
CONF=/mnt/data/bc_wifi.conf
[ -f $CONF ] && . $CONF

i=0
while [ ! -d /sys/class/net/$IF ] && [ $i -lt 20 ]; do sleep 1; i=$((i+1)); done
[ -d /sys/class/net/$IF ] || { echo "bc-net: no $IF"; exit 0; }

ip link set $IF up
iw dev $IF set power_save off 2>/dev/null

if [ -n "$SSID" ]; then
    mkdir -p /tmp/bc
    cat > /tmp/bc/wpa.conf <<WPA
ctrl_interface=/var/run/wpa_supplicant
update_config=0
network={
    ssid="$SSID"
    psk="$PSK"
}
WPA
    wpa_supplicant -B -i $IF -c /tmp/bc/wpa.conf
    n=0
    while [ $n -lt 15 ]; do
        iw dev $IF link 2>/dev/null | grep -q Connected && break
        sleep 1; n=$((n+1))
    done
    if iw dev $IF link 2>/dev/null | grep -q Connected; then
        dhcpcd -q $IF 2>/dev/null &
        echo "bc-net: STA connected"
        exit 0
    fi
    killall wpa_supplicant 2>/dev/null
fi

if command -v hostapd >/dev/null 2>&1; then
    AP_ADDR=${AP_ADDR:-192.168.50.1}
    cat > /tmp/bc/hostapd.conf <<AP
interface=$IF
ssid=${AP_SSID:-balancebot}
hw_mode=g
channel=6
wpa=2
wpa_passphrase=${AP_PSK:-balancebot}
wpa_key_mgmt=WPA-PSK
wmm_enabled=1
AP
    ip addr add $AP_ADDR/24 dev $IF
    hostapd -B /tmp/bc/hostapd.conf
    dnsmasq --interface=$IF --dhcp-range=192.168.50.20,192.168.50.60,12h --no-resolv 2>/dev/null
    echo "bc-net: AP mode ($AP_ADDR)"
else
    echo "bc-net: no STA configured and hostapd not installed (BR2_PACKAGE_HOSTAPD)"
fi
