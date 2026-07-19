# LSM6DSOX Runtime PM and System Suspend Validation

## Runtime PM design

The driver uses a 2000 ms autosuspend delay. When no hardware access or IIO
buffer is active, runtime suspend:

- masks the FIFO watermark interrupt;
- switches the FIFO to bypass and disables accel, gyro, and timestamp batching;
- writes zero to the accel and gyro ODR fields;
- disables the hardware timestamp counter;
- preserves the requested ODR, full-scale values, and FIFO watermark in the
  driver's software state.

A raw channel read, an ODR/full-scale write, or IIO buffer preenable calls
`pm_runtime_resume_and_get()`. Runtime resume restores BDU, both cached sensor
configurations, and the timestamp engine before allowing the operation to
continue. Direct operations mark the device busy and return their reference
with `pm_runtime_put_autosuspend()`.

Buffer preenable holds one runtime PM reference for the complete buffer
lifetime. This prevents autosuspend while FIFO samples are required. Buffer
postdisable returns that reference and allows the sensor to power down again.

The standard runtime PM state is visible on the I2C device:

```sh
P=/sys/bus/i2c/devices/7-006a/power
cat "$P/runtime_status"
cat "$P/runtime_usage"
cat "$P/autosuspend_delay_ms"
cat "$P/control"
```

`iio-sensor-proxy` periodically reads IIO raw channels on the LubanCat image.
Each read correctly obtains and returns a PM reference, but also refreshes the
last-busy time. Stop the service temporarily when validating that an otherwise
idle sensor reaches `runtime_status=suspended`:

```sh
sudo systemctl stop iio-sensor-proxy.service
```

## System suspend and resume

System sleep uses `pm_runtime_force_suspend()` and
`pm_runtime_force_resume()`. Before suspend, the driver disables the physical
IRQ and synchronizes the IIO pollfunc IRQ so no queued FIFO drain can access
hardware after power-down.

If the IIO buffer is active, its logical enabled state remains unchanged while
the hardware FIFO is stopped. Resume restores the cached ODR and full-scale
registers, reprograms the watermark and batching, resets the hardware timestamp
mapping, restarts continuous FIFO mode, and finally unmasks INT1. The driver
increments `hwfifo_discontinuity_count`; it does not invent samples for the
system-sleep interval.

If the sensor was already runtime-suspended before system sleep, system resume
still performs one immediate runtime resume and self-check. This avoids making
the first later ROS or sysfs access responsible for detecting a transient I2C
resume failure. Register restore is retried a bounded number of times before an
error is returned.

## LubanCat-3 validation

- Board: LubanCat-3 RK3576
- Kernel: `6.1.99-rk3576`
- System sleep mode: `deep`
- RTC wake interval: 8 seconds
- Sensor: I2C7 address `0x6a`, IRQ 107

Runtime PM was validated with `iio-sensor-proxy` temporarily stopped:

1. After 2 seconds idle, `runtime_status=suspended`, `runtime_usage=0`, and
   `CTRL1_XL`, `CTRL2_G`, and `CTRL10_C` were all `0x00`.
2. Reading `in_accel_x_raw` changed the state to `active` and restored the
   registers to `0x40`, `0x40`, and `0x20` at the default 104 Hz ranges.
3. Three seconds later the device was suspended and all three power-related
   registers were `0x00` again.
4. Enabling the IIO buffer held `runtime_usage=1`; the device remained active
   after four seconds and delivered 20 FIFO scans. Buffer disable returned the
   usage count to zero and allowed autosuspend.
5. Three repeated enable/read/disable cycles produced usage transitions of
   `1 -> 0` every time, ended suspended, and produced no runtime PM usage-count
   underflow warning.

Two system sleep paths were validated with `rtcwake -m mem -s 8`:

- With `buffer=0`, the driver logged `system suspended: buffer=0` and
  `system resumed: buffer=0 fifo=0`. A subsequent 52 Hz sysfs write woke the
  device successfully, and it autosuspended again.
- With `buffer=1`, the requested state was deliberately changed to 52 Hz,
  accel +/-16 g, gyro +/-2000 dps, and watermark 4 before suspend. After resume:
  `runtime_status=active`, `runtime_usage=1`, and `buffer=1`; `CTRL1_XL=0x34`,
  `CTRL2_G=0x3c`, `FIFO_CTRL3=0x33`, `FIFO_CTRL4=0x46`, and `INT1_CTRL=0x08`.

The active-buffer resume test reported:

```text
hwfifo_discontinuity_count=1
hwfifo_overflow_count=0
hwfifo_i2c_error_count=0
hwfifo_timestamp_backward_count=0
hwfifo_timestamp_gap_count=0
hwfifo_faulted=0
```

Reading 150 scans after resume crossed the 128-frame userspace backlog. The
physical IRQ count increased by 22, and new samples continued at approximately
19.516 ms intervals, matching 52 Hz.

After testing, the board was restored to 104 Hz, accel +/-2 g, gyro +/-250 dps,
watermark 4, and `buffer=0`. The temporarily stopped `iio-sensor-proxy` service
was restored to its original active state.
