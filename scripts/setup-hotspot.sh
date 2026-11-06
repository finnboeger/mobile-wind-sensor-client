#!/bin/bash
# Prepare a Raspberry Pi OS (Bookworm or newer) system for the windbot hotspot.
#
# The windbot client creates and opens the access point itself through NetworkManager
# (see src/dashboard/wifi.py). This script only installs what that needs and makes
# the hotspot local-only:
# - installs dnsmasq-base, which NetworkManager uses as DHCP server for the hotspot,
#   and iw to count the connected devices
# - lets devices on the hotspot reach the dashboard as http://windbot.lan
# - stops NetworkManager from forwarding traffic from the hotspot to the internet
#   (e.g. over 4G), so phones don't use up the windbot's mobile data
# - sets DNS servers that work over 4G too, while wlan0 runs the hotspot
#
# The post-first-boot script of setup.sh and update.sh run it, by hand:
# sudo ./scripts/setup-hotspot.sh
# It is safe to run again.

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
	echo "Please run as root: sudo $0" >&2
	exit 1
fi

if ! systemctl is-active --quiet NetworkManager; then
	echo "NetworkManager is not running. The hotspot needs NetworkManager to manage the Wi-Fi." >&2
	exit 1
fi

export DEBIAN_FRONTEND=noninteractive
missing=()
dpkg -s dnsmasq-base >/dev/null 2>&1 || missing+=(dnsmasq-base)
command -v iw >/dev/null 2>&1 || missing+=(iw)
command -v nft >/dev/null 2>&1 || missing+=(nftables)
if [[ ${#missing[@]} -gt 0 ]]; then
	apt-get update
	apt-get -y install "${missing[@]}"
fi

# Name for the dashboard on the hotspot, served by NetworkManager's dnsmasq
mkdir -p /etc/NetworkManager/dnsmasq-shared.d
cat <<'CONF' > /etc/NetworkManager/dnsmasq-shared.d/windbot.conf
# Written by mobile-wind-sensor-client/scripts/setup-hotspot.sh
address=/windbot.lan/10.42.0.1
address=/windbot/10.42.0.1
CONF

# Block forwarding to and from the hotspot's network (HOTSPOT_ADDRESS/24 in
# src/dashboard/wifi.py). NetworkManager's shared mode enables IP forwarding and NAT
# to the default route, which would hand out the windbot's internet connection. The
# rules are loaded at boot before the network starts, so they are in place before
# the hotspot comes up, and the windbot refuses to open the hotspot without them.
mkdir -p /etc/windbot
cat <<'NFT' > /etc/windbot/hotspot.nft
# Written by mobile-wind-sensor-client/scripts/setup-hotspot.sh
# (the first line creates the table if needed, so it can always be deleted)
table inet windbot_hotspot
delete table inet windbot_hotspot
table inet windbot_hotspot {
	chain forward {
		type filter hook forward priority -10; policy accept;
		ip saddr 10.42.0.0/24 drop
		ip daddr 10.42.0.0/24 drop
	}
}
NFT
cat <<'UNIT' > /etc/systemd/system/windbot-hotspot-firewall.service
# Written by mobile-wind-sensor-client/scripts/setup-hotspot.sh
[Unit]
Description=Block forwarding from the windbot hotspot
DefaultDependencies=no
Wants=network-pre.target
Before=network-pre.target NetworkManager.service
# nftables.service starts with "flush ruleset", which removes the table too: load it
# after nftables and again whenever nftables is restarted or reloaded
After=nftables.service
PartOf=nftables.service
ReloadPropagatedFrom=nftables.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/sbin/nft -f /etc/windbot/hotspot.nft
ExecReload=/usr/sbin/nft -f /etc/windbot/hotspot.nft

[Install]
WantedBy=multi-user.target
UNIT
systemctl daemon-reload
systemctl enable windbot-hotspot-firewall.service
systemctl restart windbot-hotspot-firewall.service

# DNS servers that don't depend on the Wi-Fi. NetworkManager only writes the DNS of
# the networks it manages to /etc/resolv.conf, and the 4G modem's usb0 isn't one of
# them. Without these, the windbot can't resolve any name (e.g. the MQTT broker) once
# wlan0 runs the hotspot or loses its network, although usb0 still has a route. They
# replace the DNS servers of every network, so names only a Wi-Fi's router knows no
# longer resolve. Only written if missing, so servers changed by hand are kept.
DNS_CONF=/etc/NetworkManager/conf.d/90-windbot-dns.conf
if [[ ! -e $DNS_CONF ]]; then
	cat <<'CONF' > "$DNS_CONF"
# Written by mobile-wind-sensor-client/scripts/setup-hotspot.sh
[global-dns-domain-*]
servers=1.1.1.1,8.8.8.8
CONF
fi

nmcli radio wifi on
systemctl reload NetworkManager

echo "Done. The windbot opens the hotspot when no known Wi-Fi network is in reach."
