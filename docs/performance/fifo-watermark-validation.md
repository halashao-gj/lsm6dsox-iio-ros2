# LSM6DSOX FIFO Watermark Validation

## FIFO v1 test environment

- Board: LubanCat-3, RK3576, aarch64
- Kernel: `6.1.99-rk3576`
- Sensor: LSM6DSOX on I2C adapter 7, address `0x6a`
- Interrupt: GPIO0_21, Linux IRQ 107, rising edge
- Accelerometer and gyroscope ODR: 104 Hz
- IIO scan: accel XYZ, gyro XYZ, software timestamp (24 bytes, historical v1)
- Capture duration: 5 seconds per case

The two cases used the same ODR, scan layout, reader and system. Only the
IIO `buffer/watermark` value changed. I2C messages were counted by summing the
`nr_msgs` field of the `i2c:i2c_result` tracepoint filtered to adapter 7.

## FIFO v1 results

| Hardware watermark | Captured scans | IRQ count | IRQ/s | I2C messages | I2C messages/s |
|---:|---:|---:|---:|---:|---:|
| 1 | 529 | 516 | 103.2 | 2081 | 416.2 |
| 8 | 524 | 57 | 11.4 | 525 | 105.0 |

Changing the watermark from 1 to 8 reduced the measured interrupt count by
88.9% and the measured I2C message count by 74.8%, while preserving an
effective output rate of approximately 104 scans/s.

The reduction is not exactly `104 / 8 = 13 IRQ/s` because the FIFO handler can
drain more than one watermark when scheduling or I2C latency allows additional
samples to arrive before the FIFO status is read.

## FIFO v1 correctness checks

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

## FIFO v1 implementation notes

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

## FIFO v2 changes

The follow-up FIFO revision changes the default watermark from 8 to 4, reducing
the 104 Hz full-batch interval from about 77 ms to about 38.5 ms. Watermark 2
remains available for lower-latency control workloads.

- The FIFO data burst target increases from 28 to 112 bytes and is clamped to
  the I2C adapter's advertised read limit, then aligned to complete 7-byte
  tagged entries. A normal four-scan batch therefore fits in one data read.
- The threaded handler uses the IIO poll function's timestamp instead of a
  shared IRQ timestamp field, removing a possible overwrite race.
- FIFO draining was bounded by a scan budget instead of an arbitrary
  four-pass limit. Normal batches do not issue a redundant second status read;
  delayed handlers recheck only when at least two complete batches accumulated.
- The optional eager `hwfifo_flush_to_buffer` callback is intentionally omitted.
  A one-frame userspace read otherwise makes the IIO core query FIFO status
  between every watermark interrupt, adding bus traffic without reducing the
  configured batch interval in the continuous reader.
- Duplicate, incomplete and unknown FIFO tags are counted. FIFO overflow, I2C
  error, tag error and selected burst-size counters are readable in buffer
  sysfs attributes.
- Invalid watermarks are rejected, and the configured watermark cannot change
  while the hardware buffer is active.

## FIFO v2 watermark 2 versus 4

Both cases used 104 Hz ODR and a five-second continuous userspace reader. The
FIFO-path I2C numbers below include only the IIO IRQ thread, excluding buffer
configuration and unrelated devices on adapter 7.

| Watermark | Userspace scans | Driver scans | IRQ | IRQ/s | FIFO I2C calls | FIFO I2C messages |
|---:|---:|---:|---:|---:|---:|---:|
| 2 | 516 | 520 | 260 | 52.0 | 520 | 1040 |
| 4 | 516 | 520 | 130 | 26.0 | 260 | 520 |

Watermark 4 halves the interrupt and FIFO bus-call rates compared with
watermark 2. Each interrupt now performs exactly two I2C calls: one FIFO status
read and one aligned bulk data read. The driver reported zero overflow, I2C and
tag errors in both runs.

Before removing the eager userspace-triggered FIFO status query and redundant
normal-batch status confirmation, the same watermark-4 test recorded 543 total
adapter-7 I2C calls and 1079 messages. The refined path recorded 277 calls and
547 messages, reductions of 49.0% and 49.3%, while retaining the same sample
rate and error-free result.

Fifty repeated watermark-4 buffer enable/disable cycles completed with the
buffer disabled. A watermark write attempted while the buffer was active was
rejected with `-EBUSY` and did not change the configured value.

The final module was also checked outside 104 Hz. At 52 Hz it queued 80 scans
with 20 IRQs and a 19.488 ms mean timestamp interval. At 208 Hz an 80-frame
userspace read caused 88 complete scans to be queued with 22 IRQs and a 4.873 ms
mean interval. Both runs reported zero FIFO overflow, I2C error and tag error;
the board was then restored to coherent 104 Hz accel/gyro ODR with its buffer
disabled.

## FIFO v3 hardware timestamps and recovery

FIFO v3 enables the LSM6DSOX hardware timestamp engine and batches one
timestamp tag with each coherent accel/gyro scan. Each combined scan therefore
occupies three 7-byte FIFO entries. The maximum user watermark is 170 scans so
that the programmed threshold remains within the 512-entry hardware FIFO and
the 9-bit watermark field.

At probe, `INTERNAL_FREQ_FINE=-11` calibrated the nominal 25 us hardware tick
to 25,412 ns. The driver resets the hardware counter when enabling or recovering
the FIFO, records the matching IIO clock reference, extends 32-bit rollover and
converts each tag to nanoseconds. It discards a complete batch if its timestamp
count does not match the paired accel/gyro count or a non-rollover tick moves
backward.

| ODR | Frames | Mean delta | Minimum | Maximum | Timestamp/tag errors |
|---:|---:|---:|---:|---:|---:|
| 26 Hz | 40 | 39.033 ms | 39.033 ms | 39.033 ms | 0 |
| 52 Hz | 40 | 19.516 ms | 19.516 ms | 19.516 ms | 0 |
| 104 Hz | 40 | 9.758 ms | 9.758 ms | 9.758 ms | 0 |
| 208 Hz | 40 | 4.879 ms | 4.879 ms | 4.879 ms | 0 |

The current 104 Hz, watermark-4 five-second run captured 512 userspace scans
with 130 IRQs, 260 FIFO-thread I2C calls and 520 I2C messages. A normal IRQ
still performs one status read and one 84-byte data read; adding the third
timestamp entry did not increase the I2C call count. All overflow, I2C, tag and
discontinuity counters remained zero.

### Forced-overflow validation

At 208 Hz, INT1 routing was deliberately masked for two seconds. FIFO status
reached `0xea00`, including the overrun flag. After restoring INT1, the driver:

1. detected the overrun and did not push the damaged FIFO contents;
2. masked INT1 and switched FIFO to bypass;
3. disabled sensor and timestamp batching;
4. reprogrammed watermark, batching and the hardware timestamp reference;
5. restored continuous mode and INT1, then delivered 20 new frames.

The resulting counters were `overflow=1`, `unknown_loss=1`,
`discontinuity=1`, `recovery=1` and `recovery_failure=0`. The exact number of
overwritten scans is intentionally reported as unknown rather than replaced by
interpolated samples.

The ROS 2 publisher now publishes `lsm6dsox/FIFO` on `/diagnostics`. A clean run
reported OK with all counters zero. The injected overflow produced WARN with
the recovery and unknown-loss counters; a failed recovery would produce ERROR
and report that INT1 remains masked.

Twenty repeated hardware-timestamp FIFO enable/disable cycles completed with
the buffer disabled. A watermark of 171 was rejected against the dynamic
maximum of 170, and the board was restored to coherent 104 Hz accel/gyro ODR.

## Remaining work

- Measure IRQ-thread CPU usage, scheduling latency and power at watermarks
  1, 2, 4, 8 and 16.
- Add adapter-level I2C fault injection and verify the recovery-failure ERROR
  diagnostic.
- Run continuously for at least 30 hours to exercise a real 32-bit hardware
  timestamp rollover.
