# Module Deployment With modprobe

Date validated: 2026-07-10

This project can be installed as a normal Linux kernel module on the LubanCat
board instead of being loaded manually with `insmod`.

## Install

Build the module on the host:

```sh
make -C lsm6dsox_driver
```

Copy it to the board:

```sh
scp lsm6dsox_driver/lsm6dsox_driver.ko lubancat3:/tmp/
```

Install it into the running kernel module tree:

```sh
sudo install -D -m 0644 /tmp/lsm6dsox_driver.ko \
  /lib/modules/$(uname -r)/extra/lsm6dsox_driver.ko
sudo depmod -a
```

Load by module name:

```sh
sudo modprobe lsm6dsox_driver
```

Enable boot-time loading:

```sh
echo lsm6dsox_driver | sudo tee /etc/modules-load.d/lsm6dsox.conf
```

## Reload Existing Module

If the module is already bound to the I2C device, unbind it before removing:

```sh
echo 7-006a | sudo tee /sys/bus/i2c/drivers/my_lsm6dsox/unbind
sudo modprobe -r lsm6dsox_driver
sudo modprobe lsm6dsox_driver
```

This avoids force-removing the module and lets the Linux device model release
the driver cleanly.

## Validation

Validated board state after reboot:

```text
/etc/modules-load.d/lsm6dsox.conf = lsm6dsox_driver
lsm6dsox_driver        16384  1
filename: /lib/modules/6.1.99-rk3576/extra/lsm6dsox_driver.ko
vermagic: 6.1.99-rk3576 SMP mod_unload aarch64
```

I2C and IIO state:

```text
/sys/bus/i2c/drivers/my_lsm6dsox/7-006a
/sys/bus/iio/devices/iio:device1/name: lsm6dsox
```

Boot-time probe logs:

```text
my_lsm6dsox 7-006a: WHO_AM_I=0x6c
my_lsm6dsox 7-006a: IIO trigger registered
my_lsm6dsox 7-006a: IIO triggered buffer setup complete
my_lsm6dsox 7-006a: data-ready irq registered: irq=107
my_lsm6dsox 7-006a: IIO device registered
```

Triggered-buffer validation:

```text
trigger0/name: lsm6dsox-dev1
/dev/iio:device1 read 48 bytes = 2 frames * 24 bytes
INT1 data-ready interrupt enabled
data-ready irq count=1 ... 10
INT1 data-ready interrupt disabled
```
