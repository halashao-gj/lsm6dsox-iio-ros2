# LSM6DSOX：从 SMBus 到 regmap 的重构记录

## 目标

将驱动的寄存器访问从分散的 `i2c_smbus_*()` 调用迁移到标准的
`regmap-i2c` 接口，同时保持 IIO channel、采样频率、INT1 生命周期和
triggered buffer 的外部行为不变。

## 迁移映射

| 原始访问 | regmap 访问 | 使用位置 |
| --- | --- | --- |
| `i2c_smbus_read_byte_data` | `regmap_read` | WHO_AM_I、配置验证 |
| `i2c_smbus_write_byte_data` | `regmap_write` | INT1 路由 |
| 手工 read-mask-write | `regmap_update_bits` | reset、BDU、ODR、量程 |
| `i2c_smbus_read_i2c_block_data` | `regmap_bulk_read` | XYZ scan 与 IIO raw channel |

`devm_regmap_init_i2c()` 在 probe 中创建寄存器映射；adapter 能力检查也从
SMBus byte/block 改为 `I2C_FUNC_I2C`，与 regmap-i2c 的底层传输方式一致。

## 缓存策略

本次显式使用 `REGCACHE_NONE`。LSM6DSOX 的 software reset 会在驱动背后重置
硬件寄存器；在尚未实现 `regcache_mark_dirty()` 与同步策略前，禁用缓存能避免
缓存值与硬件状态不一致。后续若加入 runtime PM 或 regcache，应同时定义 volatile
输出寄存器与 reset 后的 cache sync 生命周期。

## 保持不变的行为

- INT1 仍由 buffer `postenable`/`predisable` 显式启停。
- 仅 accel `DRDY_XL` 路由至 edge-triggered INT1；gyro 与 accel 继续以相同 ODR
  被同一 scan 读取。
- IIO frame 仍为 24 bytes：accel XYZ、gyro XYZ 和 timestamp。
- 现有 IIO buffer 与 ROS systemd 验收脚本仍是重构回归测试。

## 验证流程

1. 用板端对应 kernel SDK 交叉编译模块。
2. 停止服务，安装模块、`depmod` 并重启目标板，以避免在运行中替换 IIO 驱动。
3. 执行 `./scripts/validate_lsm6dsox_pipeline.sh`，确认独占读帧、INT1 连续触发、
   systemd 启动和 `/imu/data` 发布。
4. 连续执行多轮验收，确认 buffer 启停不会回归。

## 面试表述

原型阶段直接 SMBus 访问足以验证硬件；工程化阶段迁移到 regmap 可统一寄存器读写、
位更新、块读取与错误返回。这里刻意没有立刻开启缓存，因为 reset 与缓存同步是独立的
生命周期问题；先保持行为等价，再以回归验收保护后续优化。
