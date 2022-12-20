# Mobile wind monitoring system

This is the client source code for a mobile wind monitoring system. It should work with any NMEA2000 network that contains a wind and a heading sensor.
Its usefulness is primarily in allowing you to recreate something similar to the Yachtbot WindBot using only off the shelf parts and a bit of soldering for the connectors.

While the code is geared for the listed components it should be easily adaptable to other configurations.

## Bill of Materials
- approx. 3m long tube to act as a mast and attachement point
- NMEA2000 capable Wind transducer plus cable, e.g. [Garmin gWind](https://www.garmin.com/de-BE/p/144124)
- NMEA2000 capable Heading sensor plus cable, e.g. [Autonnic A5050](https://www.autonnic.com/product-page/compass-sensor-nmea2000)
- optional: NMEA2000 capable Display plus cable, e.g. [Garmin GMI 20](https://www.garmin.com/de-DE/p/126694)
- Attachment options for the components listed above
- Raspberry Pi Zero2, W version preferable for communcation options when assembled
- Waveshare Sense HAT (B), for future INS enhanced movement data
- Waveshare SIM7000G NB-IoT HAT, for GPS and cellular connection (cellular connection is a WIP and might require different hardware later)
- Waveshare RS485 CAN HAT
- 12V to 5V step down converter
- Micro USB to Micro USB OTG cable
- 22 AWG wiring to solder to panel connector
- GPIO riser 11mm body height + 6.2mm pin height from [here](https://www.berrybase.de/40-pin-gpio-stacking-header-fuer-raspberry-pi-farbig-kodiert-6-2mm), different sizes might work but have not been tested
- 8x 12mm M2.5 spacer, 4x 18mm M2.5 spacer, see [here](https://www.berrybase.de/abstandshuelse-metall-mit-gewinde-innen/aussen-m2-5)
- 4x M2.5 nut
- 4x M2.5 flat-top screw
- NMEA2000 panel connecter male, e.g. from [here](https://shop.hatlabs.fi/products/nmea-2000-panel-connector-male)
- Waterproof enclosure for Pi assembly and mounting brackets, e.g. using [these 3D Prints]()
  - Requires a smaller GPS antenna than is included, e.g. [this one](https://www.amazon.de/gp/product/B07MHGPT8L)
  - The micro usb cable needs to protrude very little when using the unmodified enclosure.
- 4 way NMEA2000 T connector
- Male and Female NMEA2000 terminators
- NMEA2000 power cable
- NMEA2000 cable to connect to the Pi
- possibly shorter cables for each of the used components

## Set up instructions
These instructions are not necessarily correct for everybody, as some people might want to leave password based authentication enabled or use a different keymap. These steps would then need to be adjusted when following the instructions.

1. Flash SD Card with ****Raspberry Pi OS Lite**** using `dd if=raspios-lite.img | pv -s 2G | sudo dd of=dev/sdX bs=4M`
2. Mount SD Card
3. `touch $SDCARD/boot/ssh`  to enable SSH on first boot
4. `echo $HOSTNAME > $SDCARD/etc/hostname` to set the hostname
5. `sed -i "s/#\{0,1\}PasswordAuthentication yes/PasswordAuthentication no/" $SDCARD/etc/ssh/sshd_config` to disable password based authentication
6. `mkdir $SDCARD/home/pi/.ssh` (Steps 6.-11.: setup public key to be able to connect via ssh)
7. `chmod 700 $SDCARD/home/pi/.ssh`
8. `chown 1000 $SDCARD/home/pi/.ssh`
9. `cp $SSHPUBKEY $SDCARD/home/pi/.ssh`
10. `chmod 600 $SDCARD/home/pi/.ssh/$SSHPUBKEY`
11. `chown 1000 $SDCARD/home/pi/.ssh/$SSHPUBKEY`
12. `sed -i "s/#dtparam=i2c_arm=on/dtparam=i2c_arm=on/" $SDCARD/boot/config.txt` Enable I2C
13. `sed -i "s/#dtparam=spi=on/dtparam=spi=on/" $SDCARD/boot/config.txt` Enable SPI
14. `echo "dtparam=act_led_trigger=actpwr\nenable_uart=1\ndtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000\ndtparam=i2c_baudrate=400000" >> $SDCARD/boot/config.txt` Enable UART, Power LED, CAN Interface, increase I2C rate
15. `sed -i "s/console=serial0,115200 //" $SDCARD/boot/cmdline.txt` Disable serial console
16. `sed -i "s/# de_DE.UTF-8 UTF-8/de_DE.UTF-8 UTF-8/" $SDCARD/etc/locale.gen` Enable german locale
17. `sed -i "s/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/" $SDCARD/etc/locale.gen` Enable US locale
18. `echo "Europe/Berlin" > $SDCARD/etc/timezone` Set Timezone
19. `echo "de-latin1" > $SDCARD/etc/vconsole.conf` Set Keymap for vconsole
20. `echo "ip link set up can0 type can bitrate 250000" >> $SDCARD/etc/rc.local` Enable CAN interface on boot.
21. `echo '\n\nnetwork={\n\tssid="$SSID"\n\tpsk="$PASSWORD"\n\tid_sts="$ID"\n}' >> $SDCARD/etc/wpa_supplicant/wpa_supplicant.conf` as always, replace the variables with the correct values.

Insert the SD Card into the Pi and connect it to power. It should automatically connnect to your Wi-Fi network and you should be able to connect to it with `ssh`. When connected execute the next steps.
1. `locale-gen` Generate locales
2. `sed -i "s/en_GB/en_US/" $SDCARD/etc/default/locale` Change default locale to en_US, personal preference
3. `sudo apt update`
4. `sudo apt upgrade`
5. `sudo apt install zsh tmux libi2c-dev python3-smbus2 python3-serial can-utils python3-venv python3-setuptools libconfig-general-perl jq make gcc python3-nmea2 python3-paho-mqtt python3-numpy git`
6. `pip install icm20948` Install library for motion sensor
7. `git clone https://github.com/finnboeger/NMEA2000.git && cd NMEA2000 && python setup.py build && python setup.py install && cd ..` Install NMEA2000 library
8. `git clone https://github.com/finnboeger/mobile-wind-sensor-client.git`
9. `sudo echo 'sudo -u pi -- bash -c "cd /home/pi/mobile-wind-sensor-client/ && python main.py"' >> /etc/rc.local` Run client code on boot
10. Create a `config.ini` in `mobile-wind-sensor-client` that contains the information for your mqtt broker in the following format: 
```
[MQTT]
BROKER = 
PORT = 
CLIENT_ID = 
TOPIC = 
USER = 
PASS = 
```
