#!/bin/bash

set -euo pipefail

usage() {
	cat <<'EOF'
Usage: ./setup.sh [options]

Options:
	-i, --input-file PATH     Path to Raspberry Pi OS image file
	-o, --output-device DEV   Output block device (for example /dev/sdb)
	-m, --sdcard PATH         Mounted SD card root path (optional)
	-n, --hostname NAME       Hostname to write
	-k, --ssh-pubkey PATH     Path to public SSH key file
	-l, --keyboard-layout VAL Keyboard layout (default: de-latin1)
	-c, --wifi-country CODE   WiFi country code (default: DE)
	-s, --ssid NAME           WiFi SSID
	-p, --password VALUE      WiFi password
	-P, --user-password VALUE System user password (username: pi)
	-y, --yes                 Skip interactive confirmations
  -h, --help                Show this help
EOF
}

prompt_if_empty() {
	local var_name="$1"
	local prompt_text="$2"
	local is_secret="${3:-false}"
	local value="${!var_name:-}"

	if [[ -z "$value" ]]; then
		if [[ "$is_secret" == "true" ]]; then
			read -r -s -p "$prompt_text: " value
			echo
		else
			read -r -p "$prompt_text: " value
		fi
	fi

	printf -v "$var_name" '%s' "$value"
}

prompt_password_with_confirmation() {
	local var_name="$1"
	local prompt_text="$2"
	local value="${!var_name:-}"
	local first
	local second

	if [[ -n "$value" ]]; then
		return 0
	fi

	while true; do
		read -r -s -p "$prompt_text: " first
		echo
		read -r -s -p "Confirm $prompt_text: " second
		echo

		if [[ -z "$first" ]]; then
			echo "$prompt_text must not be empty."
			continue
		fi

		if [[ "$first" != "$second" ]]; then
			echo "Passwords do not match. Please try again."
			continue
		fi

		printf -v "$var_name" '%s' "$first"
		break
	done
}

confirm_or_exit() {
	local question="$1"
	local answer

	if [[ "$ASSUME_YES" == "true" ]]; then
		return 0
	fi

	read -r -p "$question [y/N]: " answer
	if [[ ! "$answer" =~ ^[Yy]$ ]]; then
		echo "Aborted."
		exit 1
	fi
}

partition_path() {
	local device="$1"
	local number="$2"

	if [[ "$device" =~ [0-9]$ ]]; then
		echo "${device}p${number}"
	else
		echo "${device}${number}"
	fi
}

unmount_device_partitions() {
	local device="$1"
	local part
	local -a partitions=()

	mapfile -t partitions < <(lsblk -lnpo NAME,TYPE "$device" | awk '$2 == "part" { print $1 }')

	if [[ ${#partitions[@]} -eq 0 ]]; then
		return 0
	fi

	for ((i=${#partitions[@]}-1; i>=0; i--)); do
		part="${partitions[$i]}"
		if mount | awk -v p="$part" '$1 == p { found=1 } END { exit(found ? 0 : 1) }'; then
			echo "Unmounting $part before flashing..."
			if ! umount "$part"; then
				echo "Failed to unmount $part. Please close processes using this device and try again." >&2
				exit 1
			fi
		fi
	done
}

cleanup_mounts() {
	if [[ "$MOUNTED_BOOT" == "true" ]] && mountpoint -q "$SDCARD/boot/firmware"; then
		umount "$SDCARD/boot/firmware" || echo "Warning: failed to unmount $SDCARD/boot/firmware" >&2
	fi

	if [[ "$MOUNTED_ROOT" == "true" ]] && mountpoint -q "$SDCARD"; then
		umount "$SDCARD" || echo "Warning: failed to unmount $SDCARD" >&2
	fi

	if [[ "$AUTO_SDCARD" == "true" ]] && [[ -d "$SDCARD" ]]; then
		rmdir "$SDCARD" 2>/dev/null || true
	fi
}

INPUT_FILE="${INPUT_FILE:-}"
OUTPUT_DEVICE="${OUTPUT_DEVICE:-}"
SDCARD="${SDCARD:-}"
HOSTNAME="${HOSTNAME:-}"
SSHPUBKEY="${SSHPUBKEY:-}"
KEYBOARD_LAYOUT="${KEYBOARD_LAYOUT:-de-latin1}"
WIFI_COUNTRY="${WIFI_COUNTRY:-DE}"
SSID="${SSID:-}"
WIFI_PASSWORD="${WIFI_PASSWORD:-}"
USER_PASSWORD="${USER_PASSWORD:-}"
ASSUME_YES="false"
AUTO_SDCARD="false"
MOUNTED_ROOT="false"
MOUNTED_BOOT="false"

trap cleanup_mounts EXIT

while [[ $# -gt 0 ]]; do
	case "$1" in
		-i|--input-file|--input_file)
			INPUT_FILE="$2"
			shift 2
			;;
		-o|--output-device|--output_device)
			OUTPUT_DEVICE="$2"
			shift 2
			;;
		-m|--sdcard)
			SDCARD="$2"
			shift 2
			;;
		-n|--hostname)
			HOSTNAME="$2"
			shift 2
			;;
		-k|--ssh-pubkey|--ssh_pubkey)
			SSHPUBKEY="$2"
			shift 2
			;;
		-l|--keyboard-layout|--keyboard_layout)
			KEYBOARD_LAYOUT="$2"
			shift 2
			;;
		-c|--wifi-country|--wifi_country)
			WIFI_COUNTRY="$2"
			shift 2
			;;
		-s|--ssid)
			SSID="$2"
			shift 2
			;;
		-p|--password)
			WIFI_PASSWORD="$2"
			shift 2
			;;
		-P|--user-password|--user_password)
			USER_PASSWORD="$2"
			shift 2
			;;
		-y|--yes)
			ASSUME_YES="true"
			shift
			;;
		-h|--help)
			usage
			exit 0
			;;
		*)
			echo "Unknown argument: $1" >&2
			usage
			exit 1
			;;
	esac
done

prompt_if_empty INPUT_FILE "Path to Raspberry Pi OS image file"

if ! command -v lsblk >/dev/null 2>&1; then
	echo "lsblk is required but was not found in PATH." >&2
	exit 1
fi

if [[ -z "$OUTPUT_DEVICE" ]]; then
	echo "Available block devices:"
	lsblk
	prompt_if_empty OUTPUT_DEVICE "Choose output device (for example /dev/sdb)"
fi

echo "Confirming selected output device from lsblk:"
if ! lsblk -p | awk -v dev="$OUTPUT_DEVICE" '$1 == dev { print; found=1 } END { exit(found ? 0 : 1) }'; then
	echo "Device '$OUTPUT_DEVICE' was not found in lsblk output." >&2
	exit 1
fi
confirm_or_exit "Use output device '$OUTPUT_DEVICE'?"

prompt_if_empty HOSTNAME "Hostname"
prompt_if_empty SSHPUBKEY "Path to public SSH key"
prompt_if_empty SSID "WiFi SSID"
prompt_if_empty WIFI_PASSWORD "WiFi password" true
prompt_password_with_confirmation USER_PASSWORD "Password for the system user (pi)"

if [[ -z "$SDCARD" ]]; then
	SDCARD="$(mktemp -d /tmp/windbot-sdcard.XXXXXX)"
	AUTO_SDCARD="true"
	echo "No mount path provided. Using auto-detected mount path: $SDCARD"
fi

if [[ ! -f "$INPUT_FILE" ]]; then
	echo "Input file does not exist: $INPUT_FILE" >&2
	exit 1
fi

if [[ ! -f "$SSHPUBKEY" ]]; then
	echo "Public SSH key does not exist: $SSHPUBKEY" >&2
	exit 1
fi

if ! command -v openssl >/dev/null 2>&1; then
	echo "openssl is required but was not found in PATH." >&2
	exit 1
fi

echo "About to run: dd if='$INPUT_FILE' of='$OUTPUT_DEVICE' bs=4M status=progress"
confirm_or_exit "Proceed with flashing '$OUTPUT_DEVICE'?"
unmount_device_partitions "$OUTPUT_DEVICE"

# Flash SD Card with Raspberry Pi OS Lite
dd if="$INPUT_FILE" of="$OUTPUT_DEVICE" bs=4M status=progress

# Re-read partition table after flashing.
if command -v partprobe >/dev/null 2>&1; then
	partprobe "$OUTPUT_DEVICE"
fi

ROOT_PARTITION="$(partition_path "$OUTPUT_DEVICE" 2)"
BOOT_PARTITION="$(partition_path "$OUTPUT_DEVICE" 1)"

if [[ ! -b "$ROOT_PARTITION" ]]; then
	echo "Root partition device does not exist: $ROOT_PARTITION" >&2
	exit 1
fi

if [[ ! -b "$BOOT_PARTITION" ]]; then
	echo "Boot partition device does not exist: $BOOT_PARTITION" >&2
	exit 1
fi

mkdir -p "$SDCARD"
if ! mountpoint -q "$SDCARD"; then
	mount "$ROOT_PARTITION" "$SDCARD"
	MOUNTED_ROOT="true"
fi

mkdir -p "$SDCARD/boot/firmware"
if ! mountpoint -q "$SDCARD/boot/firmware"; then
	mount "$BOOT_PARTITION" "$SDCARD/boot/firmware"
	MOUNTED_BOOT="true"
fi

# Print each command during SD card configuration.
set -x

# Enable ssh on first boot
touch "$SDCARD/boot/firmware/ssh"

# Set the hostname
echo "$HOSTNAME" > "$SDCARD/etc/hostname"
sed -i "s/raspberrypi/$HOSTNAME/g" "$SDCARD/etc/hosts"

# Create a default user on first boot to skip interactive user setup.
USER_PASSWORD_HASH="$(openssl passwd -6 "$USER_PASSWORD")"
echo "pi:$USER_PASSWORD_HASH" > "$SDCARD/boot/firmware/userconf.txt"

# Disable password based authentication
sed -i "s/#\{0,1\}PasswordAuthentication yes/PasswordAuthentication no/" "$SDCARD/etc/ssh/sshd_config"

# Setup public key to be able to connect via ssh
mkdir -p "$SDCARD/home/pi/.ssh"
chmod 700 "$SDCARD/home/pi/.ssh"
chown 1000 "$SDCARD/home/pi/.ssh"
cat "$SSHPUBKEY" >> "$SDCARD/home/pi/.ssh/authorized_keys"
chmod 600 "$SDCARD/home/pi/.ssh/authorized_keys"
chown 1000 "$SDCARD/home/pi/.ssh/authorized_keys"

# Enable I2C
sed -i "s/#dtparam=i2c_arm=on/dtparam=i2c_arm=on/" "$SDCARD/boot/firmware/config.txt"
# Enable SPI
sed -i "s/#dtparam=spi=on/dtparam=spi=on/" "$SDCARD/boot/firmware/config.txt"
# Enable UART, Power LED, CAN Interface, increase I2C rate
printf "dtparam=act_led_trigger=actpwr\nenable_uart=1\ndtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000\ndtparam=i2c_baudrate=400000" >> "$SDCARD/boot/firmware/config.txt"

# Disable serial console
sed -i "s/console=serial0,115200 //" "$SDCARD/boot/firmware/cmdline.txt"
# Set WLAN regulatory domain in kernel cmdline.
sed -i \
	-e "s/\s*cfg80211.ieee80211_regdom=\S*//" \
	-e "s/\(.*\)/\1 cfg80211.ieee80211_regdom=$WIFI_COUNTRY/" \
	"$SDCARD/boot/firmware/cmdline.txt"
# Enable German locale
sed -i "s/# de_DE.UTF-8 UTF-8/de_DE.UTF-8 UTF-8/" "$SDCARD/etc/locale.gen"
# Enable US locale
sed -i "s/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/" "$SDCARD/etc/locale.gen"
# Set Timezone
echo "Europe/Berlin" > "$SDCARD/etc/timezone"
# Set Keymap for vconsole
echo "$KEYBOARD_LAYOUT" > "$SDCARD/etc/vconsole.conf"
XKB_LAYOUT="${KEYBOARD_LAYOUT%%-*}"
cat <<EOF > "$SDCARD/etc/default/keyboard"
XKBMODEL="pc105"
XKBLAYOUT="$XKB_LAYOUT"
XKBVARIANT=""
XKBOPTIONS=""
BACKSPACE="guess"
EOF

# autostart canbus
cat <<EOF > "$SDCARD/lib/systemd/system/canbus.service"
[Unit]
Description=Start CAN Bus
After=multi-user.target

[Service]
Type=oneshot
ExecStart=ip link set up can0 type can bitrate 250000

[Install]
WantedBy=multi-user.target
EOF

# Add post-first-boot setup script
cat <<'EOF' > "$SDCARD/home/pi/windbot-post-firstboot.sh"
#!/bin/bash

set -euox pipefail

export DEBIAN_FRONTEND=noninteractive

locale-gen de_DE.UTF-8 en_US.UTF-8
update-locale LANG=en_US.UTF-8

apt-get update
apt-get -y upgrade
apt-get -y install zsh tmux libi2c-dev can-utils python3-venv python3-setuptools libconfig-general-perl jq make gcc git

systemctl enable canbus

if [[ ! -d /home/pi/mobile-wind-sensor-client ]]; then
	sudo -u pi git clone https://github.com/finnboeger/mobile-wind-sensor-client.git /home/pi/mobile-wind-sensor-client
fi

cat <<'UNIT' > /etc/systemd/system/windbot.service
[Unit]
Description=Start wind sensor client code
After=canbus.service

[Service]
Type=simple
WorkingDirectory=/home/pi/mobile-wind-sensor-client
ExecStart=/home/pi/mobile-wind-sensor-client/.venv/bin/python src/main.py

[Install]
WantedBy=multi-user.target
UNIT

systemctl enable windbot

if [[ -f /home/pi/mobile-wind-sensor-client/config.default.ini ]] && [[ ! -f /home/pi/mobile-wind-sensor-client/config.ini ]]; then
	cp /home/pi/mobile-wind-sensor-client/config.default.ini /home/pi/mobile-wind-sensor-client/config.ini
	chown pi:pi /home/pi/mobile-wind-sensor-client/config.ini
fi

EOF
chmod 755 "$SDCARD/home/pi/windbot-post-firstboot.sh"
chown 1000:1000 "$SDCARD/home/pi/windbot-post-firstboot.sh"

# Add WiFi connection for automatic connection via NetworkManager.
NM_CONNECTION_ID="$SSID"
NM_CONNECTION_FILE="${SSID//\//_}.nmconnection"
mkdir -p "$SDCARD/etc/NetworkManager/system-connections"
cat <<EOF > "$SDCARD/etc/NetworkManager/system-connections/${NM_CONNECTION_FILE}"
[connection]
id=$NM_CONNECTION_ID
type=wifi
interface-name=wlan0

[wifi]
mode=infrastructure
ssid=$SSID

[wifi-security]
auth-alg=open
key-mgmt=wpa-psk
psk=$WIFI_PASSWORD

[ipv4]
method=auto

[ipv6]
method=auto
EOF
chmod 600 "$SDCARD/etc/NetworkManager/system-connections/${NM_CONNECTION_FILE}"

# Set country in wpa_supplicant.conf to be picked up by first-boot networking.
if [[ -f "$SDCARD/boot/firmware/wpa_supplicant.conf" ]]; then
	sed -i '/^country=/d' "$SDCARD/boot/firmware/wpa_supplicant.conf"
fi
printf "country=%s\n" "$WIFI_COUNTRY" >> "$SDCARD/boot/firmware/wpa_supplicant.conf"

# Emulate raspi-config rfkill/NetworkManager unblock behavior for first boot.
mkdir -p "$SDCARD/var/lib/NetworkManager"
cat <<EOF > "$SDCARD/var/lib/NetworkManager/NetworkManager.state"
[main]
NetworkingEnabled=true
WirelessEnabled=true
WWANEnabled=true
EOF

mkdir -p "$SDCARD/var/lib/systemd/rfkill"
for filename in "$SDCARD"/var/lib/systemd/rfkill/*:wlan; do
	if [[ ! -e "$filename" ]]; then
		continue
	fi
	echo 0 > "$filename"
done

set +x

echo "Done. SD card setup completed successfully."
