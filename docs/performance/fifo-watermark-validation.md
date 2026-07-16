# LSM6DSOX FIFO Watermark Validation

## Test environment

- Board: LubanCat-3, RK3576, aarch64
- Kernel: `6.1.99-rk3576`
- Sensor: LSM6DSOX on I2C adapter 7, address `0x6a`
- Interrupt: GPIO0_21, Linux IRQ 107, rising edge
- Accelerometer and gyroscope ODR: 104 Hz
- IIO scan: accel XYZ, gyro XYZ, software timestamp (24 bytes)
- Capture duration: 5 seconds per case

The two cases used the same ODR, scan layout, reader and system. Only the
IIO `buffer/watermark` value changed. I2C transfers were counted with the
`i2c:i2c_result` tracepoint filtered to adapter 7.

## Results

| Hardware watermark | Captured scans | IRQ count | IRQ/s | I2C transfers | I2C transfers/s |
|---:|---:|---:|---:|---:|---:|
| 1 | 529 | 516 | 103.2 | 2081 | 416.2 |
| 8 | 524 | 57 | 11.4 | 525 | 105.0 |

Changing the watermark from 1 to 8 reduced the measured interrupt count by
88.9% and the measured I2C transfer count by 74.8%, while preserving an
effective output rate of approximately 104 scans/s.

The reduction is not exactly `104 / 8 = 13 IRQ/s` because the FIFO handler can
drain more than one watermark when scheduling or I2C latency allows additional
samples to arrive before the FIFO status is read.

## Correctness checks

- The final deployed module delivered an 80-frame userspace capture at
  watermark 8 with 10 IRQs and 88 queued scans (the handler drains the complete
  final batch), zero FIFO overruns and zero I2C errors.
- At 104 Hz, reconstructed timestamps increased by 9.615 ms per scan.
- At 52 Hz, reconstructed timestamps increased by 19.231 ms per scan.
- Direct raw reads and ODR writes returned `-EBUSY` while the buffer was active.
- Writing either sampling-frequency attribute updated both accel and gyro ODRs,
  preserving coherent combined FIFO scans.
- 100 repeated buffer enable/disable cycles completed with the final buffer
  state disabled and no driver error.
- Clearing `current_trigger`, unloading and reloading the module completed
  successfully after removing the old driver's duplicate trigger release.

## Implementation notes

- INT1 now carries FIFO-watermark events instead of per-sample accel DRDY.
- The hardware FIFO stores tagged accel and gyro entries. The driver pairs one
  accel tag and one gyro tag into the existing 24-byte combined IIO scan.
- Reads are issued in 28-byte chunks, four tagged FIFO entries per I2C bulk
  transfer, instead of separate accel and gyro output-register reads per scan.
- The IRQ top half records the watermark time and polls the IIO trigger. All
  I2C access and FIFO parsing remain in the threaded IIO poll function.
- Software timestamps are reconstructed from the watermark IRQ time and ODR,
  then forced to remain monotonic across batches.
- FIFO overrun resets the FIFO stream and invalidates the timestamp base.
- A FIFO I2C read failure discards the incomplete batch, increments the error
  counter, resets the FIFO and never pushes partially initialized scans.

## Remaining work

- Measure IRQ-thread CPU usage, scheduling latency and power at watermarks
  1, 2, 4, 8 and 16.
- Add fault-injection coverage for I2C failure and forced FIFO overrun.
- Use the LSM6DSOX hardware timestamp tag to improve long-duration clock drift
  and batch-boundary timestamp accuracy.
