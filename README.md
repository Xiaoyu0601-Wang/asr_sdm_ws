# asr_sdm_radxa_ws
Workspace of Radxa Zero for Amphibious Snake-like Robot with Screw-drive Mechanism

## How to grant user permissions to utilize the spidev port
```sh
sudo groupadd spi
sudo usermod -aG spi $USER
ls -l /dev/spidev*
sudo nano /etc/udev/rules.d/99-spi.rules
SUBSYSTEM=="spidev", GROUP="spi", MODE="0660"
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo reboot
```