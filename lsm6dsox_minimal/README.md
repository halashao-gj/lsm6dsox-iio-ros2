# LSM6DSOX device tree overlay

This overlay enables I2C7-M1 on LubanCat 3 and declares an LSM6DSOX at
address `0x6a`. A student-written driver should match the compatible string
`study,lsm6dsox-minimal`.

## Build in WSL

```sh
make
```

## Install on LubanCat 3

Copy the generated `.dtbo` from WSL to the board:

```sh
scp rk3576-lubancat-lsm6dsox-i2c7-m1-overlay.dtbo lubancat3:~/
```

Then install it on the board:

```sh
sudo cp rk3576-lubancat-lsm6dsox-i2c7-m1-overlay.dtbo /boot/dtb/overlay/
sudo vi "$(readlink -f /boot/uEnv/uEnv.txt)"
```

Comment out the generic I2C7-M1 overlay and enable this combined overlay:

```ini
#dtoverlay=/dtb/overlay/rk3576-lubancat-i2c7-m1-overlay.dtbo
dtoverlay=/dtb/overlay/rk3576-lubancat-lsm6dsox-i2c7-m1-overlay.dtbo
```

Reboot and verify that Linux created the I2C device:

```sh
sync
sudo reboot
ls -l /sys/bus/i2c/devices/7-006a
tr -d '\0' < /sys/bus/i2c/devices/7-006a/of_node/compatible
```
