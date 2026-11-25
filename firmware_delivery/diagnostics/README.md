# 无桨耗时诊断镜像

本目录只用于无桨台架耗时对比，不是飞行交付基线。

| 用途 | 镜像 | 源码基准 | 字节 | SHA256 |
|---|---|---|---:|---|
| 拆分前计时 | `Robin_24G_Car_timing_before_a76d5b6.bin` | `a76d5b6` | 24660 | `3F3A26A675847AB8D86EC15F83F00430FF350815BBFFED12A2274805463208C8` |
| 拆分后计时 | `Robin_24G_Car_timing_after_e6905f0.bin` | `e6905f0` | 24788 | `5087AF437E9BCE32043FF69773F4B354C02E5C56D80C7795309544EEE5E8E7D5` |

两版均：

```text
-DC_FLIGHT_CONTROL_EN=1
-DC_FLIGHT_BARO_EN=0
-DC_FLIGHT_TIMING_DIAG=1
烧录地址 0x4000
```

## 计时定义

- `imu_sum_ticks / imu_count`：IMU 平均耗时，24 MHz tick。
- `imu_max_ticks`：IMU 最大耗时。
- 拆分前：`t3488_read_6axis` 调用前到加速度低通结束。
- 拆分后：`ImuMeasure_Update` 调用前到返回。
- `ctrl_max_ticks`：控制拍入口到 `FlightSafety_Apply` 写完 PWM 的最大耗时。
- `remain_min_ticks`：写完 PWM 后距 `next_tick` 的最小剩余时间，负值按 0 记录。
- `missed_ticks`：控制循环漏拍累计。
- `imu_count` 包含读取失败的测量尝试。

换算：

```text
us = ticks / 24
```

`u32` 的 `imu_sum_ticks` 在累计 IMU 时间约 179 秒后回绕；单次无桨测试请控制在 2 分钟内。

## RAM 字段

`g_flight_timing` 地址：

```text
拆分前：0x8418C4
拆分后：0x8418D0
```

字段偏移：

| 偏移 | 字段 | 含义 |
|---:|---|---|
| `+0x00` | `magic` | `0x54494D47`（TIMG） |
| `+0x04` | `imu_sum_ticks` | IMU 累计 tick |
| `+0x08` | `imu_max_ticks` | IMU 最大 tick |
| `+0x0C` | `imu_count` | IMU 测量次数 |
| `+0x10` | `ctrl_max_ticks` | 控制到 PWM 最大 tick |
| `+0x14` | `remain_min_ticks` | 最小剩余 tick |
| `+0x18` | `missed_ticks` | 漏拍数 |
| `+0x1C` | `seq` | 控制拍累计 |
| `+0x20` | `imu_start` | 最近一次 IMU 起点 |
| `+0x24` | `ctrl_start` | 最近一次控制起点 |

读取方式：

1. BDT 连接后读取上述 RAM 地址；
2. 先确认 `magic == 0x54494D47`；
3. 采样前后读取同一组字段，按差值计算本次测试；
4. 断电重启会清零。

本镜像未启用 GPIO 翻转；如需要硬件测点，可在 `FlightTiming_ImuBegin/End` 和 `CtrlBegin/End` 处接测试 GPIO，比较同一段代码的脉冲宽度。

## 台架步骤

1. 不装桨，烧录两份镜像分别测试。
2. 两版使用相同遥控输入和相同控制时序。
3. 静止启动，确认解锁后四路 PWM 能正常输出。
4. 做一次急停，确认 PWM 清零。
5. 断开遥控，确认失联保护。
6. 在相同时间窗口读取 RAM 统计。
7. 对比 `imu_max_ticks`、`ctrl_max_ticks`、`remain_min_ticks` 和 `missed_ticks`。

飞行基线 `firmware_delivery/candidates/Robin_24G_Car_flight_control_baseline.bin` 和普通拆分候选均未改动。
