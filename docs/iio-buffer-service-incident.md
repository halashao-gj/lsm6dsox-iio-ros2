# IIO Buffer 在 systemd 场景无持续帧：故障复盘

本记录对应 LSM6DSOX 驱动、IIO triggered buffer 与 ROS 2 systemd 服务的
一次真实问题，适合作为驱动/IIO/ROS 集成类面试的项目复盘材料。

## 现象

`lsm6dsox-ros.service` 可以正常启动：`ExecStartPre` 成功、
`buffer/enable=1`、`current_trigger=lsm6dsox-dev1`，ROS 节点也能打开
`/dev/iio:device1` 并注册 `/imu/data` publisher。但是订阅端在限定时间内
收不到任何 `sensor_msgs/msg/Imu`。

问题还具有启停相关的间歇性：一次运行可能有帧，`systemctl stop` 后下一次
启动又可能没有帧。

## 实际调试时间线

这段时间线保留了排障中出现的误判和修正。面试时可以用它说明自己如何逐步缩小
问题范围，而不是只给出最后的补丁。

1. **先确认执行环境。** 最初的本地终端是 WSL，PID 1 不是 systemd，也没有
   `/dev/iio:device1`；它不能代表板端状态。随后通过 `cat@192.168.137.24` 连接到
   LubanCat，确认远端 PID 1 为 systemd。
2. **确认 service 准备阶段正常。** `systemctl start` 成功，IIO buffer 为 `1`，
   current trigger 为 `lsm6dsox-dev1`，scan elements 全部启用。驱动日志曾显示
   data-ready IRQ 计数递增，因此没有一开始就修改 ROS QoS。
3. **识别了一个有干扰的测试。** 在服务运行时执行 `dd /dev/iio:device1` 令 ROS
   节点报 `Device or resource busy`。这说明 IIO 字符设备是独占消费者，而不是节点
   权限配置错误。此后改为“服务停止时 dd 单读帧；服务运行时订阅 ROS topic”。
4. **定位到 IIO 生命周期。** ROS 节点能够注册 publisher 并打开设备，却没有消息。
   通过 `dmesg`、`/proc/interrupts` 和新增的首帧日志，观察到问题发生在 INT1 到
   triggered buffer 的路径，而非 ROS 发布层。
5. **修复隐式硬件状态。** 将 INT1 启停从 trigger 的隐式引用计数回调移到 buffer 的
   `postenable`/`predisable`，并对 SMBus 写入重试。一次 `INT1_CTRL` 关闭返回
   `-6 (ENXIO)` 的记录表明：即使关闭失败，也必须清除软件的 `buffer_enabled`，让
   下一次启动能够重新配置硬件。
6. **修复模块生命周期。** 尝试替换模块时发现，即使 I2C 设备已解绑，模块仍显示
   “in use”。原因是 `iio_trigger_get()` 缺少对应的 `iio_trigger_put()`；修复
   `remove()` 后，第一次部署通过重启清除了旧模块泄漏。
7. **区分验收假阴性与真实驱动故障。** `ros2 topic echo --once` 的退出行为和 Python
   重定向缓冲曾导致“已收消息、脚本却超时”的假阴性；验收脚本改为
   `PYTHONUNBUFFERED=1` 并检查消息正文。之后压力测试仍出现真实失败：内核 IRQ 只到
   1–2 次就停止，这不是 CLI 问题。
8. **解决最终的硬件事件竞争。** 双 DRDY（accel 与 gyro）同时路由到 edge-triggered
   INT1 会产生紧邻边沿。由于两者 ODR 相同，最终只保留 accel `DRDY_XL` 作为完整
   scan 的采样时钟。
9. **以回归验证收尾。** 重启加载新模块后，一键脚本连续执行 10 次均通过；最后一轮
   IRQ 从 1 连续增长到至少 600，并收到了 `/imu/data`。服务由脚本清理为停止状态。

## 排查过程

1. 确认服务身份和 udev 权限。服务以 `cat` 用户运行，buffer 准备脚本以 root
   运行；这不是权限问题。
2. 检查 IIO 状态。buffer 已启用、scan elements 已启用、trigger 名称正确。
3. 区分消费者。`/dev/iio:device1` 只能由一个进程独占打开；在 ROS 节点运行时
   再执行 `dd` 会让节点报 `Device or resource busy`，因此不能把并发 `dd` 当作
   buffer 的有效测试。正确做法是停止服务后单独读帧，服务运行时订阅 ROS topic。
4. 检查内核日志和 `/proc/interrupts`。早期版本存在 data-ready IRQ 计数，而服务
   启停后 IIO trigger 没有继续得到事件，说明断点在 INT1/trigger/buffer 生命周期，
   而不是 ROS QoS。
5. 增加驱动日志后确认：修复后可以看到 `first IIO buffer frame queued`，证明路径
   已到达 `iio_push_to_buffers_with_timestamp()`。

## 根因

初版驱动把 INT1 数据就绪中断的启停完全交给 IIO trigger 的
`.set_trigger_state()`。trigger 可在 buffer 启用前就被选择，服务的
disable -> enable 循环又依赖 IIO 内部引用计数；这使硬件 INT1 的实际状态不再和
buffer 状态一一对应，后续启动可能没有重新使能 INT1。

后续压力测试还发现，初版同时把 accel 和 gyro 的 DRDY 路由到同一个
edge-triggered INT1。两者配置为相同 ODR，任意一个 accel DRDY 都足以作为读取完整
accel+gyro scan 的采样点；双路由会产生紧邻的边沿，并偶发使共享 GPIO 的事件状态异常，
表现为启动后只有 1–2 次 IRQ。最终改为只路由 `DRDY_XL`。

另一个放大问题是，关闭 INT1 的 SMBus 写入曾返回 `-6`（`ENXIO`）。旧代码只在
写入成功时才清除 `buffer_enabled` 标志。于是软件保留“已经开启”的陈旧状态，下一次
`postenable` 跳过寄存器配置，导致没有新 IRQ 和用户态帧。

驱动解绑时还遗漏了与 `iio_trigger_get()` 对应的 `iio_trigger_put()`，造成模块引用
无法释放；第一次部署修复时需要重启目标板清除旧模块泄漏。

## 修复

- 使用 `iio_buffer_setup_ops`：在 `postenable` 显式配置 INT1，在 `predisable`
  显式关闭 INT1。
- 将 INT1 配置与 trigger 选择解耦；`.set_trigger_state()` 不再隐式操作硬件。
- INT1 SMBus 写入最多重试三次；即使关闭失败，也强制清除软件状态，让下一次
  `postenable` 重新配置硬件。
- 只将 accel DRDY 路由至 INT1；在相同 ODR 下，它是 accel/gyro scan 的唯一稳定采样
  时钟。
- 计数器改为每个设备实例私有，并在首帧入 buffer 时输出调试日志。
- 在 `remove()` 中释放 trigger 引用，保证正常 `modprobe -r` 生命周期。

## 验收方式

运行：

```sh
./scripts/validate_lsm6dsox_pipeline.sh
```

脚本依次检查模块和 IIO 设备、停止服务后独占读取一个 24-byte IIO frame、启动服务、
并以 `BEST_EFFORT` QoS 确认 `/imu/data` 的实际消息内容。不要以
`ros2 topic echo --once` 的退出码作为唯一标准：在 ROS 2 Humble 中该命令收到消息后
不一定立即退出，脚本应检查捕获输出是否含有 `header:`；由于它是 Python 进程，输出
重定向到文件时还必须设置 `PYTHONUNBUFFERED=1`，否则 `timeout` 可能在缓冲区刷新前
终止进程并制造假阴性。

部署后在目标板连续运行 10 次均通过，验证了先前最容易复现的 stop/start 回归。

## 面试可讲的要点

- 我没有把“systemd active”和“ROS publisher 存在”当作数据链路成功；我把链路拆成
  IRQ、IIO trigger、buffer、字符设备与 ROS 发布逐段验证。
- 我识别到 IIO 字符设备的单消费者语义，避免用并发 `dd` 制造假性 `EBUSY`。
- 我把硬件寄存器状态的所有权从隐式 trigger 生命周期移动到显式 buffer 生命周期，
  让状态边界与用户可见的 `buffer/enable` 对齐。
- 我处理了失败路径：硬件 I2C 写失败后，软件状态不能假装仍有效；下一次启动必须能够
  自我恢复。
- 我把一次手工排障沉淀成自动验收脚本，并用连续启停验证回归。
