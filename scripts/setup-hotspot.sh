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
#
# Run it once on the Pi: sudo ./scripts/setup-hotspot.sh
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

# Block forwarding from the hotspot while it is up. NetworkManager's shared mode
# enables IP forwarding and NAT to the default route, which would hand out the
# windbot's internet connection.
cat <<'DISPATCHER' > /etc/NetworkManager/dispatcher.d/90-windbot-hotspot
#!/bin/bash
# Written by mobile-wind-sensor-client/scripts/setup-hotspot.sh
INTERFACE=$1
ACTION=$2

[[ "${CONNECTION_ID:-}" == "windbot-hotspot" ]] || exit 0

case "$ACTION" in
	up)
		nft delete table inet windbot_hotspot 2>/dev/null
		nft -f - <<NFT
table inet windbot_hotspot {
	chain forward {
		type filter hook forward priority -10; policy accept;
		iifname "$INTERFACE" drop
		oifname "$INTERFACE" drop
	}
}
NFT
		logger -t windbot-hotspot "Blocked forwarding from the hotspot on $INTERFACE"
		;;
	down)
		nft delete table inet windbot_hotspot 2>/dev/null
		logger -t windbot-hotspot "Removed the forwarding block for $INTERFACE"
		;;
esac
exit 0
DISPATCHER
chmod 755 /etc/NetworkManager/dispatcher.d/90-windbot-hotspot

nmcli radio wifi on
systemctl reload NetworkManager

echo "Done. The windbot opens the hotspot when no known Wi-Fi network is in reach."
