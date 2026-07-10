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
不一定立即退出，脚本应检查捕获输出是否含有 `header:`。

部署后在目标板连续运行两次均通过，验证了先前最容易复现的 stop/start 回归。

## 面试可讲的要点

- 我没有把“systemd active”和“ROS publisher 存在”当作数据链路成功；我把链路拆成
  IRQ、IIO trigger、buffer、字符设备与 ROS 发布逐段验证。
- 我识别到 IIO 字符设备的单消费者语义，避免用并发 `dd` 制造假性 `EBUSY`。
- 我把硬件寄存器状态的所有权从隐式 trigger 生命周期移动到显式 buffer 生命周期，
  让状态边界与用户可见的 `buffer/enable` 对齐。
- 我处理了失败路径：硬件 I2C 写失败后，软件状态不能假装仍有效；下一次启动必须能够
  自我恢复。
- 我把一次手工排障沉淀成自动验收脚本，并用连续启停验证回归。
