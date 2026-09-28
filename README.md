# SuperPower

`SuperPower` 是主控侧的超级电容 CAN 通信模块，负责接收超电状态帧、同步裁判系统功率上限，
并把控制帧发回超电控制板。

这个模块只做通信和状态缓存，不负责电机限幅计算，也不负责 UI 绘制。功率控制由
`PowerControl` 使用这里提供的实测功率和在线状态完成，底盘 UI 只读取这里的只读状态。

## 职责

- 接收超电状态帧 `0x051`
- 解析底盘功率、裁判系统总功率、最大输出功率和输出能力
- 订阅 `chassis_ref` 话题，缓存裁判系统底盘功率上限
- 收到有效状态帧后，在 CAN 接收回调里按 `5 ms` 最小间隔发送控制帧 `0x061`
- 使用连续相同状态帧计数判断离线，不使用定时超时检测

模块没有线程和定时器，所有收发都在 CAN 接收回调中完成。

## 协议

| 项目 | 约定 |
|---|---|
| CAN 类型 | Classic CAN |
| 帧格式 | 标准帧 |
| 数据长度 | 8 字节 |
| 字节序 | 小端（与 STM32 本地字节序一致） |
| 状态帧 ID | `0x051`，超电到主控 |
| 控制帧 ID | `0x061`，主控到超电 |

代码使用 packed 结构体描述 8 字节数据区，并通过 `LibXR::Memory::FastCopy` 在 CAN 数据区和
结构体之间拷贝。

## 状态帧

状态帧由超电控制板发送给主控，标准帧 ID 为 `0x051`。

```cpp
struct __attribute__((packed)) StatusData {
  uint8_t power_limit;
  uint16_t chassis_power;
  uint16_t referee_power;
  uint16_t superpower_output_max;
  uint8_t output_capability;
};
```

| 偏移 | 字段 | 类型 | 含义 |
|---:|---|---|---|
| 0 | `power_limit` | `uint8_t` | 超电侧当前功率限制原始值 |
| 1 | `chassis_power` | `uint16_t` | 底盘实际功率编码值 |
| 3 | `referee_power` | `uint16_t` | 裁判系统总输出功率编码值 |
| 5 | `superpower_output_max` | `uint16_t` | 超电可向 A 侧输出的最大功率 |
| 7 | `output_capability` | `uint8_t` | 输出能力原始值，范围 `0~255` |

`chassis_power` 和 `referee_power` 需要按下面公式解码后再对外提供：

```cpp
power_w = (static_cast<float>(encoded) - 16384.0f) / 64.0f;
```

`superpower_output_max` 按直接功率值读取，不使用零点偏移公式。

当前代码不做功率滤波，收到新状态帧后直接缓存解码结果。模块离线时，功率接口返回 `0`。
长度不足 8 字节的状态帧直接丢弃。

## 控制帧

控制帧由主控发送给超电控制板，标准帧 ID 为 `0x061`。

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

控制帧不单独开线程发送。收到有效状态帧且模块在线时，接收回调会检查距离上一次发送是否已经
达到 `5 ms`，满足条件才发送一帧。

## 在线判定

模块启动后，在收到第一帧有效状态帧之前认为离线。

每次收到有效状态帧时，会和上一帧完整的 `StatusData` 比较：

- 数据变化时，连续相同帧计数重置为 `1`
- 数据相同时，连续相同帧计数加 `1`
- 连续相同帧数量达到 `200` 时认为离线，并清空对外状态（不清空缓存的裁判功率上限）

这里不使用最后接收时间做超时判断。

## 运行流程

1. 构造时在传入的 CAN 总线上注册标准帧过滤器，只接收 ID `0x051`
2. 查找 `chassis_ref` 话题并注册回调，保存裁判系统底盘功率上限；话题不存在时触发 `ASSERT`
3. CAN 接收回调检查状态帧长度，长度不足 8 字节时丢弃
4. 有效状态帧进入重复帧计数和协议解析
5. 在线时按 `5 ms` 最小间隔发送控制帧

## 对外接口

| 接口 | 在线返回 | 离线返回 |
|---|---|---|
| `GetChassisPower()` | 解码后的底盘实际功率，单位 W | `0` |
| `GetRefereePower()` | 解码后的裁判系统总输出功率，单位 W | `0` |
| `GetSuperPowerOutputMax()` | 最大输出功率，单位 W | `0` |
| `GetPowerLimit()` | 超电侧功率限制原始值 | `0` |
| `GetCapEnergy()` | `output_capability / 255.0f` | `0` |
| `IsOnline()` | `true` | `false` |

`GetCapEnergy()` 是为兼容旧上层命名保留的接口，当前表示输出能力比例，不表示电容容量或
剩余电量。

## 依赖

- `QDU-Robomaster/Referee`：提供 `Referee::ChassisPack` 类型，并创建本模块订阅的 `chassis_ref`
  话题。

无外部软件包依赖。

## 构造接口

```cpp
SuperPower(LibXR::CAN& can_bus);
```

依赖项：

- `can_bus`：`LibXR::CAN`，连接超级电容控制板的 CAN 总线。

无配置项。

## 使用

```sh
xrobot module add QDU-Robomaster/SuperPower
xrobot setup
xrobot instance add QDU-Robomaster/SuperPower
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空；把 `can_bus` 填为 BSP
中用 `XR_REGISTER` 注册的 CAN 对象名：

```yaml
modules:
  - module: QDU-Robomaster/SuperPower
    id: superpower_0
    args:
      - can_bus: can1
```

BSP 侧：

```cpp
XR_REGISTER(can1, LibXR::CAN);
```

构造时必须已经存在 `chassis_ref` 话题，因此 `QDU-Robomaster/Referee` 实例（使用默认的
`referee_chassis_tp_name`）必须在 `modules:` 中排在本实例之前。该话题没有新数据时，控制帧里的
`referee_power_limit` 保持初始值 `0` 或上一次缓存值。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/SuperPower`
（在 BSP 中）打印当前的构造函数。
