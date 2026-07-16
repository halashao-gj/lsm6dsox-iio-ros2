# LSM6DSOX Dynamic Scale Validation

## IIO ABI

The writable IIO `scale` value is the SI conversion factor for one raw LSB,
not the full-scale range number. Query the exact accepted values before writing:

```sh
D=/sys/bus/iio/devices/iio:device1
cat "$D/in_accel_scale_available"
cat "$D/in_anglvel_scale_available"
```

The driver reports:

| Sensor range | IIO scale | Register FS value |
|---|---:|---:|
| Accel +/-2 g | `0.000598205` m/s^2/LSB | `CTRL1_XL[3:2]=0` |
| Accel +/-4 g | `0.001196411` m/s^2/LSB | `CTRL1_XL[3:2]=2` |
| Accel +/-8 g | `0.002392822` m/s^2/LSB | `CTRL1_XL[3:2]=3` |
| Accel +/-16 g | `0.004785645` m/s^2/LSB | `CTRL1_XL[3:2]=1` |
| Gyro +/-125 dps | `0.000076358` rad/s/LSB | `CTRL2_G.FS_125=1` |
| Gyro +/-250 dps | `0.000152716` rad/s/LSB | `CTRL2_G[3:2]=0` |
| Gyro +/-500 dps | `0.000305432` rad/s/LSB | `CTRL2_G[3:2]=1` |
| Gyro +/-1000 dps | `0.000610865` rad/s/LSB | `CTRL2_G[3:2]=2` |
| Gyro +/-2000 dps | `0.001221729` rad/s/LSB | `CTRL2_G[3:2]=3` |

For example:

```sh
echo 0.004785645 | sudo tee "$D/in_accel_scale"
echo 0.001221729 | sudo tee "$D/in_anglvel_scale"
```

Scale writes claim IIO direct mode. They return `-EBUSY` while the triggered
buffer is active. A supported write updates only the target sensor's FS bits,
reads the control register back, and updates the cached scale only after the
readback matches. A write or verification failure attempts to restore the old
register value.

## Board validation

- Board: LubanCat-3 RK3576
- Kernel: `6.1.99-rk3576`
- Sensor: LSM6DSOX on I2C7 address `0x6a`
- Default configuration restored after testing: +/-2 g, +/-250 dps

All nine scales were written through IIO sysfs and read back. `CTRL1_XL` values
were `0x40`, `0x48`, `0x4c`, and `0x44` for +/-2/4/8/16 g at 104 Hz.
`CTRL2_G` values were `0x42`, `0x40`, `0x44`, `0x48`, and `0x4c` for
+/-125/250/500/1000/2000 dps at 104 Hz.

An unsupported accel scale returned `-EINVAL` and preserved the previous
+/-16 g value. Attempts to change either scale with the FIFO buffer enabled
returned `-EBUSY`. The FIFO still delivered 20 unchanged 24-byte frames at
+/-16 g and +/-2000 dps.

The ROS 2 publisher was started at those maximum ranges and read:

```text
accel_scale=0.004785645 gyro_scale=0.001221729
```

This confirms that range selection changes conversion metadata and control
registers without changing the IIO scan layout or ROS message path.
