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
