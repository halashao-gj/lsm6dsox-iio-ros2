# LSM6DSOX 驱动架构与代码阅读指南

本文从项目分层、内核对象、调用链和关键函数四个角度解释当前驱动。建议打开
`lsm6dsox_driver/lsm6dsox_driver.c`，按本文给出的顺序同步阅读。

## 1. 项目整体分层

```text
设备树 overlay
  compatible / I2C 地址 / GPIO 中断
                |
                v
Linux I2C driver + regmap
  probe / 寄存器配置 / FIFO / IRQ / PM
                |
                v
Linux IIO core
  sysfs raw/scale/ODR + trigger + /dev/iio:deviceX
                |
         +------+------+
         |             |
         v             v
 IIO-raw 测试程序   ROS 2 imu_publisher
                       |
                       v
             /imu/data + /diagnostics
```

- `lsm6dsox_minimal/` 负责让内核发现 I2C7 上地址 `0x6a` 的设备，并提供
  INT1 GPIO 中断。
- `lsm6dsox_driver/` 是本文重点：把芯片寄存器能力包装为标准 IIO ABI。
- `IIO-raw/` 直接读取 24 字节扫描帧，用于隔离验证内核驱动。
- `lsm6dsox_ros/` 读取同一 IIO buffer，根据 IIO scale 转成 SI 单位并发布
  ROS 2 消息。
- `scripts/`、`systemd/` 和 `config/udev/` 负责部署及用户态生命周期，不参与
  中断处理。

## 2. 首先认识三个核心对象

### 2.1 `struct lsm6dsox_data`：设备运行状态

见驱动约第 149 行。它由 `iio_priv(indio_dev)` 分配，每个传感器实例拥有一份。

成员可分为六组：

1. 总线对象：`client`、`regmap`。
2. IIO对象：`trig`、`lock`。
3. 用户请求的配置：`accel_odr`、`gyro_odr`、两个 `scale_nano`、
   `fifo_watermark`。
4. 硬件运行状态：`buffer_enabled` 表示 IIO 逻辑状态，`fifo_running` 表示硬件
   是否真的运行。这两个状态在 system suspend 时会暂时不同。
5. 时间戳状态：上次硬件 tick、32 位回绕次数、tick 到 ns 的参考点、上一帧
   时间戳。
6. 诊断计数：IRQ、样本、overflow、I2C错误、tag错误、丢帧、不连续、恢复、
   时间戳倒退/跳变/回绕。

普通状态在 `data->lock` 下修改；可在 IRQ/诊断读取之间共享的计数使用
`atomic_t` 或 `atomic64_t`。

### 2.2 `struct lsm6dsox_scan`：送给用户态的一帧

见约第 189 行。内容是：

```text
accel_x/y/z：3 * s16 = 6 字节
gyro_x/y/z： 3 * s16 = 6 字节
对齐填充：               4 字节
timestamp：     s64 = 8 字节
总计：                  24 字节
```

量程和 ODR 不在扫描帧内。量程改变的是 raw 到 SI 单位的换算系数，因此 ROS
节点只需在启动时读取 IIO scale，不需要修改 24 字节协议。

### 2.3 `iio_dev`：驱动与 IIO core 的连接点

`probe()` 中填写：

- `indio_dev->channels`：六个轴和一个软件时间戳通道；
- `indio_dev->info`：raw、scale、ODR、available、watermark 回调；
- `indio_dev->trig`：当前驱动自己的 FIFO watermark trigger；
- `available_scan_masks`：要求 accel XYZ 和 gyro XYZ 一起进入 buffer。

IIO core 根据这些描述自动创建 sysfs 属性和 `/dev/iio:deviceX`。

## 3. 建议的代码阅读顺序

不要从文件第一行一路读到底。按下面六条调用链阅读更容易建立整体模型。

## 4. 调用链一：设备如何被发现和初始化

入口在 `lsm6dsox_driver`（约第 2127 行）：

```text
设备树 compatible
  -> of_match
  -> I2C core 调用 probe()
  -> 建立 regmap 和 IIO device
  -> WHO_AM_I 验证
  -> 软件复位 / BDU / timestamp / 默认 ODR和量程
  -> 注册 IIO trigger、triggered buffer、物理 IRQ
  -> 注册 IIO device
  -> 启用 runtime PM
```

重点阅读 `lsm6dsox_probe()`（约第 1885 行）：

1. `i2c_check_functionality()` 拒绝不支持完整 I2C transfer 的适配器。
2. 根据 adapter quirks 计算 `fifo_read_chunk`，最后向下对齐到 7 字节 FIFO
   entry，避免一次 burst 在 tag 中间截断。
3. `devm_regmap_init_i2c()` 建立寄存器访问层。这里使用 `REGCACHE_NONE`，因为
   软件复位和 PM 会在 regmap 不知情时改变硬件状态，驱动选择显式恢复配置。
4. 预分配 FIFO raw、解析后 scan、硬件时间戳三个数组，避免中断线程中分配。
5. 检查 `WHO_AM_I == 0x6c`，然后初始化硬件。
6. `devm_*` 资源由设备模型自动释放，减少 probe 失败路径的手工回滚。
7. 最后设置 2 秒 autosuspend；probe 返回后设备会从默认 104 Hz进入 ODR=0。

## 5. 调用链二：sysfs raw、ODR 和量程

IIO core 通过 `lsm6dsox_iio_info`（约第 1751 行）进入驱动。

### 5.1 raw 读取

`cat in_accel_x_raw` 的路径：

```text
IIO sysfs
  -> lsm6dsox_read_raw(RAW)
  -> iio_device_claim_direct_mode()
  -> pm_runtime_resume_and_get()
  -> mutex
  -> regmap_bulk_read(2 bytes)
  -> 解码 little-endian s16
  -> pm_runtime_put_autosuspend()
```

`claim_direct_mode()` 是 direct/buffer 互斥的关键。buffer 开启时该函数返回
`-EBUSY`，防止 sysfs 单次读与 FIFO 同时消费或修改硬件。

### 5.2 ODR 写入

`echo 52 > in_accel_sampling_frequency` 进入 `lsm6dsox_write_raw()`，然后调用
`lsm6dsox_set_odr()`（约第 1177 行）。

- 先从 `lsm6dsox_odr_table` 找到 52 Hz 对应的寄存器位；
- 同步修改 accel 和 gyro，保证 FIFO 两种 tag 保持同速；
- 写完回读验证；
- 任一写入失败时恢复两个传感器的旧 ODR。

### 5.3 量程写入

量程路径进入 `lsm6dsox_set_full_scale()`（约第 1238 行）：

- accel 和 gyro 分别选择量程表、寄存器和 mask；
- IIO `scale` 是一个 raw LSB 对应的 SI 值，不是直接写 `16g` 或 `2000dps`；
- 只有回读位完全匹配后才更新软件缓存；
- 写入或验证失败时，使用旧表项回滚寄存器。

`read_avail()` 把表导出成 `*_available`，用户态应先读取可用值再写。

## 6. 调用链三：buffer 如何启动和停止

`lsm6dsox_buffer_ops`（约第 1012 行）定义四阶段生命周期：

```text
preenable
  -> 获取 runtime PM 引用，确保硬件已唤醒
postenable
  -> 清诊断状态
  -> FIFO bypass
  -> 开启 accel/gyro/timestamp batching
  -> 写 watermark
  -> 重置硬件 timestamp
  -> FIFO continuous
  -> 打开 INT1 watermark 路由

predisable
  -> 关闭 INT1
  -> FIFO bypass
  -> 关闭所有 batching
  -> 清逻辑运行状态
postdisable
  -> 归还 runtime PM 引用，允许 autosuspend
```

真正操作硬件的公共函数是 `lsm6dsox_fifo_start_hw()` 和
`lsm6dsox_fifo_stop_hw()`（约第 473、521 行）。PM resume 也复用 start 函数，
从而保证普通 enable 和系统唤醒使用同一套寄存器顺序。

一个 combined scan 包含三个 7 字节硬件 entry：gyro、accel、timestamp。因此
watermark 4 scan 会被转换为 12 entry；104 Hz 时理想 watermark IRQ 约为
`104 / 4 = 26 Hz`。

## 7. 调用链四：一次 FIFO 中断如何变成用户态数据

```text
LSM6DSOX FIFO达到watermark
  -> INT1物理IRQ
  -> lsm6dsox_irq_handler()
  -> iio_trigger_poll()
  -> IIO threaded pollfunc
  -> lsm6dsox_trigger_handler()
  -> mutex
  -> lsm6dsox_fifo_drain()
  -> 读FIFO状态和tagged entries
  -> 组装scan并转换timestamp
  -> iio_push_to_buffers_with_timestamp()
  -> /dev/iio:deviceX
```

物理 IRQ 顶半部（约第 339 行）只计数并触发 IIO poll，不进行 I2C，因为 I2C
操作可以睡眠。真正的 FIFO I2C 读取在 threaded handler 中进行。

`lsm6dsox_fifo_drain()`（约第 797 行）负责：

- 读取 FIFO level、empty、watermark、overrun；
- 把 entry 数量向下取整为完整 scan；
- 限制单次处理量，防止无限占用线程；
- 必要时重新读取状态，处理 handler 执行期间新到达的数据。

`lsm6dsox_fifo_read_entries()`（约第 667 行）是数据路径核心：

1. 按 adapter 允许的 chunk 做多个连续 burst read。
2. 读取每个 entry 的 tag，分别解析 gyro、accel、timestamp。
3. accel 和 gyro 都到齐时组成一个 `lsm6dsox_scan`。
4. 批次存在重复、未知、残缺 tag，或 scan 数与 timestamp 数不一致时，整批
   丢弃并恢复 FIFO，绝不把错位数据送给用户态。
5. 每帧写入独立时间戳后调用 IIO push。

## 8. 调用链五：硬件时间戳

LSM6DSOX timestamp 是 32 位 tick。`lsm6dsox_init_hw_timestamp()` 读取芯片的
`INTERNAL_FREQ_FINE`，在标称 25 us/tick 基础上校正实际 tick 周期。

`lsm6dsox_hw_timestamp_to_ns()`（约第 644 行）执行：

```text
raw 32-bit tick
  -> 与上一个tick比较
  -> 大幅变小：视为32位回绕，wraps++
  -> 小幅倒退：视为错误，拒绝批次
  -> extended_tick = wraps << 32 | raw
  -> timestamp_ns = reference_ns + extended_tick * tick_ns
```

之后还会检查：

- 当前时间戳必须严格大于上一帧；
- 相邻间隔超过两个 ODR 周期时增加 gap 和 discontinuity；
- overflow、FIFO恢复、system suspend 后清除 timestamp valid，明确开启新时间段，
  不用插值伪造休眠期间的样本。

## 9. 调用链六：错误恢复

`lsm6dsox_fifo_recover()`（约第 546 行）的顺序是：

```text
标记 discontinuity / recovery / faulted
  -> 屏蔽 INT1
  -> FIFO bypass
  -> 关闭 batching
  -> 重写 watermark
  -> 恢复 batching
  -> 重置 timestamp
  -> FIFO continuous
  -> buffer仍启用时重新打开 INT1
  -> 清除 faulted
```

FIFO status 出现 overrun 时增加 `overflow` 和 `unknown_loss`。因为硬件只能说明
发生覆盖，无法证明具体丢了多少帧，所以驱动不报告虚假的精确 loss 数量。

如果恢复本身失败，驱动保持 INT1 屏蔽、FIFO bypass 和 `fifo_faulted=1`，避免
中断风暴或继续输出边界不可信的数据。

## 10. runtime PM 和 system suspend

### 10.1 runtime PM

`lsm6dsox_runtime_suspend()`（约第 1760 行）最终调用
`lsm6dsox_power_down()`：停止 FIFO、把两个 ODR 位写 0、关闭 timestamp。

`lsm6dsox_runtime_resume()`（约第 1790 行）从软件缓存恢复 BDU、ODR、量程和
timestamp。I2C恢复最多重试三次；如果逻辑 buffer 仍开启，还会重启 FIFO。

### 10.2 system suspend

`lsm6dsox_suspend()` 先关闭物理 IRQ，再 `synchronize_irq()` 等待已经排队的 IIO
threaded handler 完成，最后调用 `pm_runtime_force_suspend()`。

`lsm6dsox_resume()` 调用 `pm_runtime_force_resume()`，恢复完成后再打开 Linux
IRQ。即使设备在 system sleep 前已经 runtime-suspended，也会主动做一次恢复
自检，然后重新进入 autosuspend，避免第一次 ROS 访问才暴露恢复错误。

关键设计是：

- `buffer_enabled` 是用户/IIO 的逻辑状态；
- `fifo_running` 是瞬时硬件状态；
- system suspend 时可以出现 `buffer_enabled=1, fifo_running=0`；
- resume 后恢复为 `buffer_enabled=1, fifo_running=1`。

## 11. remove 和资源释放

`lsm6dsox_remove()` 先唤醒设备并禁用 runtime PM，再在锁内执行 power-down。
其他由 `devm_*` 创建的 IIO device、trigger、IRQ和内存由设备模型按逆序释放。

## 12. 阅读时重点思考的五个问题

1. 为什么物理 IRQ 顶半部不能直接调用 `regmap_bulk_read()`？
2. 为什么 watermark 4 要写入 12 个 FIFO entry，而不是 4？
3. 为什么量程回读成功之前不能更新 `accel_scale_nano`？
4. 为什么 system suspend 后只增加 discontinuity，而不能补齐缺失样本？
5. 为什么 `buffer_enabled` 与 `fifo_running` 必须是两个变量？

能用自己的话回答这五题，就已经掌握了当前驱动最关键的并发、数据边界、IIO
ABI、错误恢复和电源管理设计。

## 13. 当前明确不做的范围

- 不实现设备树安装方向矩阵；固定安装时由 ROS TF 或上层节点处理坐标变换。
- runtime PM 使用 ODR=0 进入芯片低功耗，当前设备树没有 regulator 电源轨控制。
- 驱动采用显式寄存器恢复，不使用 regcache 同步。
- 仍需更长时间的真实 32 位 timestamp 回绕测试和专门的 I2C fault injection。
