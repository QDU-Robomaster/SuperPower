# SuperPower

`SuperPower` 是机器人主控侧的超级电容 CAN 通信模块。它负责接收超电状态帧、同步裁判系统功率上限、定时下发超电控制帧，并向 `PowerControl` 和底盘 UI 提供只读状态。

本模块是 BSP 侧驱动，不是超电控制板固件。本文所有收发方向都以主控为视角。

## 职责边界

- 接收超电状态帧 `0x051`，解析功率、输出能力和在线状态。
- 订阅裁判系统 `chassis_ref` 话题，缓存底盘功率上限。
- 在线时按最小 `5 ms` 间隔发送超电控制帧 `0x061`。
- 对上层只暴露解码后的功率值和必要状态，不暴露旧协议状态字、错误等级、功率级开关等接口。

`PowerControl` 当前通过 `GetChassisPower()` 获取实测底盘功率，用于功率模型更新和输出限幅。

## 协议约定

| 项目 | 约定 |
|---|---|
| CAN 类型 | Classic CAN |
| 帧类型 | 标准帧 |
| 数据长度 | 8 字节 |
| 字节序 | STM32 本地小端 |
| 状态帧 ID | `0x051`，超电 -> 主控 |
| 控制帧 ID | `0x061`，主控 -> 超电 |

代码使用 `__attribute__((packed))` 描述 8 字节协议布局，并用 `memcpy` 在 CAN 数据区和协议结构体之间转换。

## 状态帧

状态帧由超电控制板发送给主控，CAN 标准帧 ID 为 `0x051`。

```cpp
struct __attribute__((packed)) StatusData {
  uint8_t power_limit;
  uint16_t chassis_power;
  uint16_t referee_power;
  uint16_t superpower_output_max;
  uint8_t output_capability;
};
```

| 偏移 | 字段 | 类型 | 对外语义 |
|---:|---|---|---|
| 0 | `power_limit` | `uint8_t` | 超电侧当前功率限制原始值 |
| 1 | `chassis_power` | `uint16_t` | 解码后为底盘实际功率，单位 W |
| 3 | `referee_power` | `uint16_t` | 解码后为裁判系统总输出功率，单位 W |
| 5 | `superpower_output_max` | `uint16_t` | 超电可向 A 侧输出的最大功率，单位 W |
| 7 | `output_capability` | `uint8_t` | 输出能力原始值，范围 `0~255` |

`chassis_power` 和 `referee_power` 不是直接功率值，必须先按下面公式解码：

```cpp
power_w = (static_cast<float>(encoded) - 16384.0f) / 64.0f;
```

也就是：

- 编码值 `16384` 对应 `0 W`。
- 编码值每增加 `64`，功率增加 `1 W`。
- 模块离线时，功率接口返回 `0`，不会把清零后的编码值解码成负功率。

`superpower_output_max` 是超电计算出的最大可输出功率，按直接功率值读取，不使用零点偏移公式。

`GetCapEnergy()` 只是兼容旧上层命名，当前实际语义是输出能力比例：

```cpp
output_capability / 255.0f
```

它不表示电容容量，也不表示剩余电量。

## 控制帧

控制帧由主控发送给超电控制板，CAN 标准帧 ID 为 `0x061`。

```cpp
struct __attribute__((packed)) CommandData {
  uint8_t flags;
  uint16_t referee_power_limit;
  uint16_t reserved0;
  uint8_t reserved1;
  int16_t reserved2;
};
```

| 偏移 | 字段 | 类型 | 当前写入 |
|---:|---|---|---|
| 0 | `flags` | `uint8_t` | `bit0` 固定置 `1`，使能 `enableCONV` |
| 1 | `referee_power_limit` | `uint16_t` | `chassis_ref.rs.chassis_power_limit` |
| 3 | `reserved0` | `uint16_t` | `0` |
| 5 | `reserved1` | `uint8_t` | `0` |
| 6 | `reserved2` | `int16_t` | `0` |

`flags` 没有使用 C 位域映射，代码通过 `ENABLE_CONV_MASK = 0x01` 显式设置 `bit0`。

## 运行机制

1. 构造时根据 `can_bus_name` 查找 CAN 总线。
2. 注册标准帧过滤器，只接收 ID `0x051`。
3. 订阅 `chassis_ref` 话题，缓存裁判系统底盘功率上限。
4. CAN 接收回调只把最新状态帧放入长度为 `1` 的无锁队列。
5. 定时任务每 `2 ms` 执行一次 `Update()`，从队列取出状态帧并解析。
6. 在线时按最小 `5 ms` 间隔发送 `0x061` 控制帧。
7. 超过 `1.0 s` 没有收到新状态帧时判定离线，并清空对外状态数据。

状态帧 `dlc` 小于 `sizeof(StatusData)` 时会被丢弃。

## 对外接口

| 接口 | 在线返回 | 离线返回 |
|---|---|---|
| `GetChassisPower()` | 低通滤波后的 `chassis_power`，单位 W | `0` |
| `GetRefereePower()` | 低通滤波后的 `referee_power`，单位 W | `0` |
| `GetSuperPowerOutputMax()` | `superpower_output_max`，单位 W | `0` |
| `GetPowerLimit()` | `power_limit` 原始值 | `0` |
| `GetCapEnergy()` | `output_capability / 255.0f` | `0` |
| `GetOutputCapabilityRaw()` | `output_capability` 原始值 | `0` |
| `IsOnline()` | `true` | `false` |

功率接口返回的是结算后的功率，不是 CAN 帧里的编码值。

## YAML 配置

最小配置如下：

```yaml
- id: superpower
  name: SuperPower
  constructor_args:
    can_bus_name: can1
```

配置要求：

- `can_bus_name` 必须对应 `User/app_main.cpp` 中已经注册的 CAN 设备。
- 系统中需要存在 `Referee` 模块，并持续发布 `chassis_ref` 话题，控制帧中的裁判功率上限才会实时更新。
- YAML 构造参数必须和 `SuperPower` 构造函数保持一致；当前只需要 `can_bus_name`。

## 模块声明

Required Hardware:

- can

Depends:

- qdu-future/Referee

代码入口：

- `Modules/SuperPower/SuperPower.hpp`
