# Mobile wind monitoring system

## Data logging & replay

The client can record raw inputs and computed outputs to allow working on the
processing logic without needing to collect live data.

- Enable data logging via `LOGGING.DATA_LOG_DIR` (one rotating log file per stream).
- Streams recorded include `gps` (raw), `position`, `heading`, `wind` (apparent) and `true_wind`.

Replay with a terminal UI:

- `python src/replay.py` (uses paths from `config.ini`)
- `python src/replay.py data_logs` (directory containing per-stream `*.log*` files)

Controls: `space` play/pause, `n` step (paused), `+/-` change speed, `q` quit.

## Bill of Materials
TODO

### Used Modules & How they connect:
- [Waveshare SIM7600G-H 4G HAT (B)](https://www.waveshare.com/wiki/SIM7600G-H_4G_HAT_(B))
  - Uses USB via POGO Pins
- [Waveshare RS485 CAN HAT](https://www.waveshare.com/wiki/RS485_CAN_HAT)
  - Uses SPI with CE0 chip selector for CAN
  - Would use UART (GPIO 14, 15) for RS485 if we were using it
- [Waveshare SX1262 433M LoRa HAT](https://www.waveshare.com/wiki/SX1262_868M_LoRa_HAT)
  - Uses UART (GPIO 14, 15)
- [Sparkfun GPS Dead Reckoning breakout board using u-blox NEO-M8U](https://www.sparkfun.com/sparkfun-gps-dead-reckoning-breakout-neo-m8u-qwiic.html)
  - Supports SPI (via Jumper), UART and I2C


## Set up instructions
These instructions are not necessarily correct for everybody, as some people might want to leave password based authentication enabled or use a different keymap. These steps would then need to be adjusted when following the instructions.

The instructions have been tested with RaspiOS Lite Bookworm

### Prepare Raspberry Pi
Download the Raspberry Pi OS Lite image from https://www.raspberrypi.com/software/operating-systems/, then run the `setup.sh` script, which handles all relevant steps.
In summary, it will
- write the image to the sdcard
- set username and password
- enable ssh and copy the public key
- enable I2C, SPI, UART and CAN
- set locale, timezone and keymap
- setup the wifi

After the script has run, insert the SD Card into the Raspberry Pi, power it on and connect to it using `ssh` and the user `pi`. Then run the `windbot-post-firstboot.sh` script that can be found in the pi users home directory. This script will
- generate the locales
- update the installed packages
- install zsh, tmux, libi2c-dev, can-utils, python3-venv, python3-setuptools, libconfig-general-perl, jq, make, gcc and git
- enable the canbus on system start
- create and enable a systemd service to start the windbot code
- copy `config.default.ini` to `config.ini` in `mobile-wind-sensor-client`

Lastly as a manual step, the values in the `config.ini` need to be set.

Personal preference: Install oh-my-zsh and set theme
1. `sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"`
2. `sed -i 's/ZSH_THEME="robbyrussell"/ZSH_THEME="agnoster"/g' .zshrc`

### Prepare for use with multiple interfaces
This is mainly relevant when developing, as in that case you're likely to have the device connected to Wi-Fi for SSH access but still want to use the wwan modem for testing purposes.
It might also be the case that you want to use a Wi-Fi hotspot instead of the modem.
In either case you need to configure the desired interface in the config.ini

On the linux side of things you need to configure it for multi-interface operation by running the following script:
```bash
cat << "EOF"  | sudo tee /etc/NetworkManager/dispatcher.d/99-modem-route
#!/bin/bash

INTERFACE=$1
ACTION=$2
TABLE_ID=200
TARGET_METRIC=800

# Only run for eth0 when it connects or updates DHCP
if [ "$INTERFACE" = "eth0" ]; then
    if [ "$ACTION" = "up" ] || [ "$ACTION" = "dhcp4-change" ]; then

        # 1. Get current IP and Gateway of the modem
        # We use ip command as it's reliable.
        IP_ADDR=$(ip -4 addr show eth0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}')

        # Extract Gateway (usually x.x.x.1)
        # We look for the default route associated with eth0
        GATEWAY=$(ip route show dev eth0 | grep default | awk '{print $3}')

        # Fallback: if no default route found (due to metric), assume gateway is X.X.X.1
        if [ -z "$GATEWAY" ]; then
            GATEWAY=$(echo "$IP_ADDR" | awk -F. '{print $1"."$2"."$3".1"}')
        fi

        # 2. Safety Check: Ensure we have values
        if [ -n "$IP_ADDR" ] && [ -n "$GATEWAY" ]; then

            # 3. Clean up old rules to prevent duplicates (ignore errors)
            # We try to delete any rule pointing to table 200 to be safe
            ip rule show | grep "lookup $TABLE_ID" | while read -r line; do
                # Extract the source IP from the rule and delete it
                SRC=$(echo "$line" | grep -oP '(?<=from\s)\d+(\.\d+){3}')
                if [ -n "$SRC" ]; then
                    ip rule del from "$SRC" table $TABLE_ID
                fi
            done

            # 4. Apply the Policy Routing (Table 200) for specific binding
            ip rule add from "$IP_ADDR" table $TABLE_ID
            ip route add default via "$GATEWAY" dev eth0 table $TABLE_ID

            # 5. Fix Main Routing Priority
            # Remove the automatic default route (usually metric 100) and re-add it as backup (metric 800)
            # This ensures wlan0 (metric 600) remains the primary default gateway.
            ip route del default dev eth0 2>/dev/null
            ip route add default via "$GATEWAY" dev eth0 metric $TARGET_METRIC 2>/dev/null

            # Log to syslog for debugging (view with: journalctl -t "modem-route")
            logger -t "modem-route" "Applied routing: IP $IP_ADDR via GW $GATEWAY table $TABLE_ID (Metric set to $TARGET_METRIC)"
        fi
    fi

    # Cleanup when interface goes down
    if [ "$ACTION" = "down" ]; then
         ip rule show | grep "lookup $TABLE_ID" | while read -r line; do
            SRC=$(echo "$line" | grep -oP '(?<=from\s)\d+(\.\d+){3}')
            if [ -n "$SRC" ]; then
                ip rule del from "$SRC" table $TABLE_ID
            fi
        done
    fi
fi
EOF

sudo chmod +x /etc/NetworkManager/dispatcher.d/99-modem-route
```
Reboot afterward to let NetworkManager run the script and apply the rules

### Prevent WiFi interface from going into powersaving mode
By default the Raspberry Pi has its WiFi interface going into power saving mode if there is no activity.
This causes unresponsiveness with running SSH sessions unless a command is actively being typed or some output is getting sent.
To avoid this, disable powersaving by running the following command:
```bash
cat << EOF | sudo tee /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf
[connection]
wifi.powersave = 2
EOF
```
Then restart NetworkManager using `sudo systemctl restart NetworkManager`.

Verify its success using `sudo iw dev wlan0 get power_save`
