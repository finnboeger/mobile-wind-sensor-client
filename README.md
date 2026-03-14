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
1. Flash SD Card with ****Raspberry Pi OS Lite**** using `sudo dd if=raspios-lite.img of=dev/sdX bs=4M status=progress`
2. Mount SD Card
3. `touch $SDCARD/boot/ssh`  to enable SSH on first boot
4. `echo $HOSTNAME > $SDCARD/boot/firmware/hostname` to set the hostname
5. `sed -i "s/#\{0,1\}PasswordAuthentication yes/PasswordAuthentication no/" $SDCARD/etc/ssh/sshd_config` to disable password based authentication
6. Setup public key to be able to connect via ssh
    1. `mkdir $SDCARD/home/pi/.ssh`
    2. `chmod 700 $SDCARD/home/pi/.ssh`
    3. `chown 1000 $SDCARD/home/pi/.ssh`
    4. `cat $SSHPUBKEY >> $SDCARD/home/pi/.ssh/authorized_keys`
    5. `chmod 600 $SDCARD/home/pi/.ssh/$SSHPUBKEY`
    6. `chown 1000 $SDCARD/home/pi/.ssh/$SSHPUBKEY`
7. `sed -i "s/#dtparam=i2c_arm=on/dtparam=i2c_arm=on/" $SDCARD/boot/firmware/config.txt` Enable I2C
8. `sed -i "s/#dtparam=spi=on/dtparam=spi=on/" $SDCARD/boot/firmware/config.txt` Enable SPI
9. `echo "dtparam=act_led_trigger=actpwr\nenable_uart=1\ndtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000\ndtparam=i2c_baudrate=400000" >> $SDCARD/boot/firmware/config.txt` Enable UART, Power LED, CAN Interface, increase I2C rate
10. `sed -i "s/console=serial0,115200 //" $SDCARD/boot/firmware/cmdline.txt` Disable serial console
11. `sed -i "s/# de_DE.UTF-8 UTF-8/de_DE.UTF-8 UTF-8/" $SDCARD/etc/locale.gen` Enable german locale
12. `sed -i "s/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/" $SDCARD/etc/locale.gen` Enable US locale
13. `echo "Europe/Berlin" > $SDCARD/etc/timezone` Set Timezone
14. `echo "de-latin1" > $SDCARD/etc/vconsole.conf` Set Keymap for vconsole
15. Create `$SDCARD/lib/systemd/system/canbus.service` with content
    ```ini
    [Unit]
    Description=Start CAN Bus
    After=multi-user.target

    [Service]
    Type=oneshot
    ExecStart=ip link set up can0 type can bitrate 250000

    [Install]
    WantedBy=multi-user.target
    ```
16. `echo '\n\nnetwork={\n\tssid="$SSID"\n\tpsk="$PASSWORD"\n\tid_sts="$ID"\n}' >> $SDCARD/etc/wpa_supplicant/wpa_supplicant.conf` as always, replace the variables with the correct values.

Insert the SD Card into the Pi and connect it to power. It should automatically connect to your Wi-Fi network and you should be able to connect to it with `ssh`. When connected execute the next steps.
1. `sudo dpkg-reconfigure locales` Verify correct locales are ticked and generate locales
2. `sed -i "s/en_GB/en_US/" /etc/default/locale` Change default locale to en_US, personal preference
3. `sudo apt update`
4. `sudo apt upgrade`
5. `sudo apt install zsh tmux libi2c-dev can-utils python3-venv python3-setuptools libconfig-general-perl jq make gcc git`
6. Personal preference: Install oh-my-zsh and set theme
    1. `sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"`
    2. `sed -i 's/ZSH_THEME="robbyrussell"/ZSH_THEME="agnoster"/g' .zshrc`
7. `sudo systemctl enable canbus` enable canbus on start
8. `git clone https://github.com/finnboeger/mobile-wind-sensor-client.git`
9. Create `/lib/systemd/system/windbot.service` with content
    ```ini
    [Unit]
    Description=Start wind sensor client code
    After=canbus.service

    [Service]
    Type=simple
    WorkingDirectory=/home/pi/mobile-wind-sensor-client
    ExecStart=/home/pi/mobile-wind-sensor-client/.venv/bin/python src/main.py

    [Install]
    WantedBy=multi-user.target
    ```
    to run the client code on boot
10. `sudo systemctl enable windbot` enable wind sensor client on start
11. Copy the `config.default.ini` to `config.ini` in `mobile-wind-sensor-client` and set the required information for e.g. the MQTT broker
