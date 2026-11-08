# Mobile wind monitoring system

## Data logging & replay

The client can record raw inputs and computed outputs to allow working on the
processing logic without needing to collect live data.

- Enable data logging via `LOGGING.DATA_LOG_DIR` (one rotating log file per stream).
- Streams recorded include `gps` (raw), `position`, `heading`, `wind` (apparent) and `true_wind`.

Replay with a terminal UI:

- `python src/replay.py` (uses paths from `config.ini`)
- `python src/replay.py data_logs` (directory containing per-stream `*.log*` files)
- `python src/replay.py data_logs --features mqtt --current-timestamps` (publishes to
  MQTT like the windbot, but shifts every record to the current time, to test the
  uplink end to end with old data; only at 1x speed, and nothing before `--jump` is
  published)

Controls: `space` play/pause, `n` step (paused), `+/-` change speed, `q` quit.

## Local dashboard and hotspot

The windbot serves its own dashboard (`src/dashboard`), so the wind can be watched
on a phone without a trip through the internet, e.g. when there is no 4G.
It is enabled with `NETWORK.ENABLE_LOCAL_WEBSERVER` and listens on
`NETWORK.WEBSERVER_PORT` (80) on every interface.

- **Wind** shows the same values and charts as the online dashboard, from one second
  averages kept in memory for two hours. The settings are stored in the browser like
  online.
- **Live** shows the latest inputs for debugging: apparent wind, heading, GPS fix,
  what the true wind is waiting for, MQTT, network interfaces and the Pi's load,
  temperature and memory.
  Other modules can add a card by publishing values with `telemetry.update(...)`.
- **Wi-Fi** shows the connection, lets you add, edit, forget and connect to networks,
  and turn the hotspot on or off without a restart.

When the Wi-Fi isn't connected to a known network, the windbot opens an access point
(`NETWORK.AP_SSID` / `AP_PASSWORD`; `setup.sh` names it `WindSensor-XXXX` after the
end of the Pi's serial number, and generates the password and prints it). If
`AP_PASSWORD` isn't empty but has fewer than 8 or more than 63 characters, the access
point stays closed and the Wi-Fi tab shows why.
Connect to it and open
http://10.42.0.1 (or http://windbot.lan). The access point doesn't share the windbot's
internet connection.

- After startup it waits `AP_STARTUP_DELAY_SECONDS` (60) for a known network.
- If the network drops, NetworkManager keeps trying to reconnect, and the access point
  opens again after `AP_FALLBACK_DELAY_SECONDS` (180) without a connection.
- While no device uses the access point (none active in the last minute), the windbot
  closes it every 15 minutes to look for a known network, e.g. a phone hotspot
  switched on as a backup, and opens it again if none connects.
- *Turn off hotspot* closes the access point and joins the known network in reach with
  the highest priority. If none connects, the access point opens again after the
  fallback delay.
- *Connect* on a known network switches to it. If that fails, the access point opens
  again right away and the dashboard shows the error.
- *Ignore known networks (test 4G)* opens the access point right away and keeps it
  open even when a known network is in reach, so the internet connection goes over the
  4G modem. It lasts for an hour, or until you stop it, connect to a network,
  or the windbot restarts. If the access point can't be opened, it ends right away.
- `NETWORK.ADMIN_PASSWORD` is required for Wi-Fi changes, e.g. when the windbot joins
  a shared network. `setup.sh` generates it; without it anyone who can reach
  the dashboard can change the Wi-Fi, and the windbot logs a warning.

The Wi-Fi management uses NetworkManager (`nmcli`) and needs root, which the
`windbot` service has. `scripts/setup-hotspot.sh` prepares the Pi: it installs
`dnsmasq-base`, `iw` and `nftables`, adds the `windbot.lan` name and blocks forwarding
from the access point with a service that loads the rules at boot. The windbot
doesn't open the access point without that block, and closes it if the block
disappears while it is open. The post-first-boot script and
`update.sh` run it; on a git checkout run `sudo ./scripts/setup-hotspot.sh` once.

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
Download the Raspberry Pi OS Lite image from https://www.raspberrypi.com/software/operating-systems/, then run the `setup.sh` script, which handles all relevant steps. It doesn't need a clone of this repository, every release contains it:

```bash
curl -fsSLO https://github.com/finnboeger/mobile-wind-sensor-client/releases/latest/download/setup.sh
chmod +x setup.sh
sudo ./setup.sh   # writes the SD card, see ./setup.sh --help for the options
```

In summary, it will
- write the image to the sdcard
- set username and password
- enable ssh and copy the public key
- enable I2C, SPI, UART and CAN
- set locale, timezone and keymap
- setup the wifi
- generate a password for the windbot's Wi-Fi hotspot and one for changing the Wi-Fi
  on the local dashboard (`--hotspot-password` / `--admin-password` set your own,
  `--no-admin-password` leaves the dashboard without one), and print both at the end.
  Note them down; on the windbot they are only in `config.ini`.

After the script has run, insert the SD Card into the Raspberry Pi, power it on and connect to it using `ssh` and the user `pi`. Then run the `windbot-post-firstboot.sh` script that can be found in the pi users home directory. This script will
- generate the locales
- update the installed packages
- install zsh, tmux, libi2c-dev, can-utils, python3-venv, python3-setuptools, libconfig-general-perl, jq, make, gcc and git
- enable the canbus on system start
- install the latest release of the windbot code to `mobile-wind-sensor-client` (with `--git` it clones the repository instead)
- create and enable a systemd service to start the windbot code
- copy `config.default.ini` to `config.ini` in `mobile-wind-sensor-client`, readable
  only by `pi` and root, with the generated hotspot and admin passwords and the
  hotspot name `WindSensor-XXXX`, where `XXXX` is the end of the Pi's serial number
- prepare the Pi for the Wi-Fi hotspot (`scripts/setup-hotspot.sh`): install
  `dnsmasq-base`, `iw` and `nftables`, make the dashboard reachable as `windbot.lan`,
  and block forwarding from the hotspot with `windbot-hotspot-firewall.service`, so
  devices on it can't use the windbot's internet connection
- delete itself, since it contains the passwords

Lastly as a manual step, the values in the `config.ini` need to be set.

### Releases and updates
A release is published by CI whenever the `version` in `pyproject.toml` changes to one without a release. It contains the code and the built dashboard, so built files are never committed.

To update a Pi installed from a release, run `~/mobile-wind-sensor-client/update.sh` for the latest release or `update.sh 0.3.0` for a specific one. It keeps `config.ini`, the virtual environment and the logs, runs `scripts/setup-hotspot.sh` (with `sudo`) so older windbots get the hotspot setup too, and restarts the service.

A git checkout (`setup.sh --git`, for development) is updated with `git pull`. The dashboard has to be built there by hand.

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
# Interface of the 4G modem: usb0 for the SIM7600 HAT in RNDIS mode, eth0/eth1 on some setups
MODEM_INTERFACE=usb0
TABLE_ID=200
TARGET_METRIC=800

# Remove every rule that points to our table. "ip rule" prints the table by name if
# it has one in /etc/iproute2/rt_tables (e.g. "lookup modem" instead of "lookup 200"),
# so grepping its output misses them and rules pile up whenever the modem's IP changes.
# "ip rule del table N" matches by number and fails once none is left.
remove_rules() {
    while ip rule del table $TABLE_ID 2>/dev/null; do :; done
}

if [ "$INTERFACE" = "$MODEM_INTERFACE" ]; then
    if [ "$ACTION" = "up" ] || [ "$ACTION" = "dhcp4-change" ]; then

        # 1. Get the current IP of the modem interface
        IP_ADDR=$(ip -4 -o addr show "$MODEM_INTERFACE" | awk '{split($4, a, "/"); print a[1]; exit}')

        # The default route NetworkManager added (before we moved it to TARGET_METRIC)
        GATEWAY=$(ip -4 route show default dev "$MODEM_INTERFACE" | awk '{print $3; exit}')

        # Fallback: if no default route found, assume the gateway is X.X.X.1
        if [ -z "$GATEWAY" ]; then
            GATEWAY=$(echo "$IP_ADDR" | awk -F. '{print $1"."$2"."$3".1"}')
        fi

        # 2. Safety check: ensure we have values
        if [ -n "$IP_ADDR" ] && [ -n "$GATEWAY" ]; then

            # 3. Policy routing (table TABLE_ID) for traffic bound to the modem's address
            remove_rules
            ip rule add from "$IP_ADDR" table $TABLE_ID
            ip route replace default via "$GATEWAY" dev "$MODEM_INTERFACE" table $TABLE_ID

            # 4. Main table: keep the modem as backup (metric TARGET_METRIC) so wlan0
            # (metric 600) stays the primary default gateway when it is connected.
            # Add the new route before removing the others, so there is never a moment
            # without a default route.
            ip route replace default via "$GATEWAY" dev "$MODEM_INTERFACE" metric $TARGET_METRIC
            ip -4 route show default dev "$MODEM_INTERFACE" | grep -v "metric $TARGET_METRIC" | while read -r route; do
                ip route del $route dev "$MODEM_INTERFACE"
            done

            # Log to syslog for debugging (view with: journalctl -t "modem-route")
            logger -t "modem-route" "Applied routing: IP $IP_ADDR via GW $GATEWAY table $TABLE_ID (Metric set to $TARGET_METRIC)"
        fi
    fi

    # Cleanup when the interface goes down
    if [ "$ACTION" = "down" ]; then
        remove_rules
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
