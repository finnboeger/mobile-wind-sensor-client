#!/bin/bash

set -ex

# Flash SD Card with Raspberry Pi OS Lite
dd if="$INPUT_FILE" of="$OUTPUT_DEVICE" bs=4M status=progress

# TODO: mount sdcard (from $OUTPUT_DEVICE) and set $SDCARD to mount point

# Enable ssh on first boot
touch "$SDCARD/boot/ssh"

# Set the hostname
echo "$HOSTNAME" > "$SDCARD/boot/firmware/hostname"

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
# Enable German locale
sed -i "s/# de_DE.UTF-8 UTF-8/de_DE.UTF-8 UTF-8/" "$SDCARD/etc/locale.gen"
# Enable US locale
sed -i "s/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/" "$SDCARD/etc/locale.gen"
# Set Timezone
echo "Europe/Berlin" > "$SDCARD/etc/timezone"
# Set Keymap for vconsole
echo "de-latin1" > "$SDCARD/etc/vconsole.conf"

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

# Add wifi network for automatic connection
# TODO: check if this still works and probably rewrite to use networkmanager (/etc/NetworkManager/system-connections/)
printf '\n\nnetwork={\n\tssid="%s"\n\tpsk="%s"\n\tid_sts="%s"\n}' "$SSID" "$PASSWORD" "$ID" >> "$SDCARD/etc/wpa_supplicant/wpa_supplicant.conf"
