# 无桨耗时诊断镜像

本目录只用于无桨台架耗时对比，不是飞行交付基线。

| 用途 | 镜像 | 源码基准 | 字节 | SHA256 |
|---|---|---|---:|---|
| 拆分前计时 | `Robin_24G_Car_timing_before_a76d5b6.bin` | `a76d5b6` | 24660 | `8863645DCC86ACB3B8CEE8AC06426215C8D95ED348E5AC02B0DB78C948F3EE1E` |
| 拆分后计时 | `Robin_24G_Car_timing_after_e6905f0.bin` | `e6905f0` | 24804 | `A124973E3C89CF6153818D88E5D3F8AF9E4A94DC0908E20BEAB85E8D8945274B` |

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
- 两版都从 IMU 读取前开始计时。
- 成功时计到预处理和积分步长换算都完成、即将进入姿态估计的位置。
- 读失败时在进入保护处理前结束，并计入该次尝试。
- 拆分前：六轴读取前到加速度低通和积分步长换算结束。
- 拆分后：`ImuMeasure_Update` 前到其返回并完成积分步长换算。
- `ctrl_max_ticks`：控制拍入口到 `FlightSafety_Apply` 写完 PWM 的最大耗时。
- `remain_min_ticks`：写完 PWM 后距 `next_tick` 的最小剩余时间，负值按 0 记录。
- `missed_ticks`：控制循环漏拍累计。
- `imu_count` 包含读取失败的测量尝试。

换算：

```text
us = ticks / 24
```

`u32` 的 `imu_sum_ticks` 在累计 IMU 用时约 179 秒后回绕；这是累计 IMU 用时，不是测试总时长。单次无桨测试建议控制在 2 分钟内。

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

## 读取方式

1. BDT 连接后先确认 `magic == 0x54494D47`。
2. 采样前后取差值的字段是：
   - `imu_sum_ticks`
   - `imu_count`
   - `seq`
   - `missed_ticks`
3. `imu_max_ticks`、`ctrl_max_ticks`、`remain_min_ticks` 是本次上电以来的极值，直接读结束值，不做差值。
4. 断电重启会清零。
5. 两版分别重启，按相同时长和相同动作采集。

平均耗时：

```text
imu_avg_us = (imu_sum_ticks_end - imu_sum_ticks_begin)
            / (imu_count_end - imu_count_begin) / 24
```

## GPIO

本镜像未启用 GPIO 翻转；RAM 字段为主读数。若需要硬件测点，可在 `FlightTiming_ImuBegin/End` 和 `FlightTiming_CtrlBegin/End` 处接测试 GPIO，比较同一段代码的脉冲宽度。

对应源码：

```text
telink_b85m_platform_src1015\demo\vendor\Robin_24G_Car\FlightTiming.h
```

## 台架步骤

1. 不装桨，分别烧录两份镜像。
2. 两版使用相同遥控输入和相同控制时序。
3. 静止启动，确认解锁后四路 PWM 正常输出。
4. 做一次急停，确认 PWM 清零。
5. 断开遥控，确认失联保护。
6. 在相同时间窗口读取 RAM 统计。
7. 对比 `imu_max_ticks`、`ctrl_max_ticks`、`remain_min_ticks` 和 `missed_ticks`。

飞行基线 `firmware_delivery/candidates/Robin_24G_Car_flight_control_baseline.bin` 和普通拆分候选均未改动。
