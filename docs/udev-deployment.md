# LSM6DSOX IIO udev Deployment

The buffered IIO character device is created dynamically by the kernel. This
rule gives only the `iio` group read/write access when the device whose sysfs
`name` is `lsm6dsox` appears. It avoids relying on a temporary `chmod` after
each boot or module reload.

## Install on the Board

Run once on the LubanCat:

```sh
sudo groupadd --system iio 2>/dev/null || true
sudo usermod -aG iio cat
sudo install -m 0644 config/udev/99-lsm6dsox-iio.rules \
  /etc/udev/rules.d/99-lsm6dsox-iio.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=change --subsystem-match=iio
```

Log out and back in, or reboot, so the `cat` login session receives its new
`iio` group membership.

## Verify

```sh
id
ls -l /dev/iio:device1
```

Expected device permissions use `root iio` ownership and mode `crw-rw----`.
The device index is not stable, so use the node associated with the IIO device
whose `name` attribute is `lsm6dsox`.

The buffer scripts still require `sudo` because they configure privileged IIO
sysfs attributes. The ROS 2 publisher itself can open the character device as
the `cat` user after this rule is installed.

## Board Validation

Validated on the RK3576 LubanCat with Linux `6.1.99-rk3576`:

```text
/dev/iio:device1: root:iio 0660
cat groups: includes iio
```

The driver was unbound, unloaded, and loaded again. After the IIO device was
created again, udev reapplied the expected `root:iio 0660` permissions. The
`cat` user then enabled the buffer using the helper and read one 24-byte frame
from `/dev/iio:device1` without a device-node `chmod`.
