# [WIP]asr_sdm_radxa_ws
Workspace of Radxa Zero for Amphibious Snake-like Robot with Screw-drive Mechanism

### How to grant user permissions to utilize the spidev port
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

### Add .dtbo filesm
```sh
sudo nano /etc/default/u-boot

# uncomment and modify the following lines
U_BOOT_UPDATE="true"
U_BOOT_FDT_OVERLAYS="rk3568-spi3-m1-cs0-mcp2515.dtbo rk3568-i2c4-m0.dtbo"
U_BOOT_FDT_OVERLAYS_DIR="/lib/firmware/6.1.0-1025-rockchip/device-tree/rockchip/overlay"
U_BOOT_SYNC_DTBS="true"

# update u-boot
sudo u-boot-update
```

### Driver installation
Driver files are all saved here.
```sh
/lib/modules/6.1.0-1025-rockchip/kernel/drivers/net/can/spi/mcp251x.ko
/lib/modules/6.1.0-1025-rockchip/kernel/drivers/net/can/dev/can-dev.ko
sudo insmod /lib/modules/6.1.0-1025-rockchip/kernel/drivers/net/can/dev/can-dev.ko
```

Add the drivers to automatic startup list.
```sh
sudo nano /etc/modules-load.d/modules-load.d
# add the following two lines
can-dev
mcp251x.ko
```

Check if the driver is installed successfully.
```sh
lsmod | grep spidev
dmesg | grep can
dmesg | grep mcp251
```

### Interface test
```sh
sudo ifconfig can0 txqueuelen 65536
sudo ip link set can0 up type can bitrate 500000
candump can0
ip link show
ip -details -statistics link show can0
```

## ROS

### Source code compilation
```sh
colcon build --symlink-install --parallel-workers 2
```
