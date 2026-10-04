# SuperPower

超级电容 CAN 通信模块：接收超电状态帧、同步裁判系统功率上限并回发控制帧 / Supercapacitor CAN communication Module that receives the capacitor status frames, synchronizes the referee power limit and sends the control frames

## 1. 模块作用 / Purpose

SuperPower 是主控侧的超级电容 CAN 通信模块，接收超电状态帧，解析底盘功率、裁判系统总功率、最大输出功率与输出能力，订阅裁判系统底盘数据 Topic（`chassis_ref_topic_name`，默认 `chassis_ref`）并缓存底盘功率上限。收到有效状态帧且模块在线时，在 CAN 接收回调中以至少 5 ms 的间隔发送控制帧，把裁判系统底盘功率上限发给超电控制板。所有收发在 CAN 接收回调中完成。

功率控制由 `PowerControl` 使用本模块提供的实测功率与在线状态完成，底盘 UI 读取本模块的状态。对外接口如下：

| 接口 | 在线返回 | 离线返回 |
| --- | --- | --- |
| `GetChassisPower()` | 解码后的底盘实际功率，单位 W | 0 |
| `GetRefereePower()` | 解码后的裁判系统总输出功率，单位 W | 0 |
| `GetSuperPowerOutputMax()` | 最大输出功率，单位 W | 0 |
| `GetPowerLimit()` | 超电侧功率限制原始值 | 0 |
| `GetCapEnergy()` | `output_capability / 255.0f`，输出能力比例 | 0 |
| `IsOnline()` | `true` | `false` |

SuperPower is the supercapacitor CAN communication Module on the main controller. It receives the capacitor status frames, decodes the chassis power, the total referee power, the maximum output power and the output capability, subscribes to the referee chassis data Topic (`chassis_ref_topic_name`, default `chassis_ref`) and caches the chassis power limit. When a valid status frame arrives and the Module is online, it sends a control frame from the CAN receive callback at intervals of at least 5 ms, passing the referee chassis power limit to the capacitor controller. All transfers happen in the CAN receive callback.

`PowerControl` performs the power control with the measured power and the online state provided by this Module, and the chassis UI reads the Module state. The public interfaces are:

| Interface | Online | Offline |
| --- | --- | --- |
| `GetChassisPower()` | Decoded actual chassis power in W | 0 |
| `GetRefereePower()` | Decoded total referee output power in W | 0 |
| `GetSuperPowerOutputMax()` | Maximum output power in W | 0 |
| `GetPowerLimit()` | Raw power limit value of the capacitor side | 0 |
| `GetCapEnergy()` | `output_capability / 255.0f`, the output capability ratio | 0 |
| `IsOnline()` | `true` | `false` |

## 2. 通信协议 / Protocol

| 项目 | 约定 |
| --- | --- |
| CAN 类型 | Classic CAN |
| 帧格式 | 标准帧 |
| 数据长度 | 8 字节 |
| 字节序 | 小端 |
| 状态帧 ID | `0x051`，超电到主控 |
| 控制帧 ID | `0x061`，主控到超电 |

状态帧：

| 偏移 | 字段 | 类型 | 含义 |
| ---: | --- | --- | --- |
| 0 | `power_limit` | `uint8_t` | 超电侧当前功率限制原始值 |
| 1 | `chassis_power` | `uint16_t` | 底盘实际功率编码值 |
| 3 | `referee_power` | `uint16_t` | 裁判系统总输出功率编码值 |
| 5 | `superpower_output_max` | `uint16_t` | 超电可向 A 侧输出的最大功率，直接为功率值 |
| 7 | `output_capability` | `uint8_t` | 输出能力原始值，范围 0 到 255 |

`chassis_power` 与 `referee_power` 按 `power_w = (encoded - 16384.0f) / 64.0f` 解码。收到新状态帧后，解码结果直接缓存，长度不足 8 字节的状态帧被丢弃。

控制帧：

| 偏移 | 字段 | 类型 | 写入值 |
| ---: | --- | --- | --- |
| 0 | `flags` | `uint8_t` | `bit0` 置 1，使能 `enableCONV` |
| 1 | `referee_power_limit` | `uint16_t` | `chassis_ref.rs.chassis_power_limit` |
| 3 | `reserved0` | `uint16_t` | 0 |
| 5 | `reserved1` | `uint8_t` | 0 |
| 6 | `reserved2` | `int16_t` | 0 |

Topic 没有新数据时，`referee_power_limit` 保持初始值 0 或上一次缓存的值。

| Item | Convention |
| --- | --- |
| CAN type | Classic CAN |
| Frame format | Standard frame |
| Data length | 8 bytes |
| Byte order | Little endian |
| Status frame ID | `0x051`, capacitor to main controller |
| Control frame ID | `0x061`, main controller to capacitor |

Status frame:

| Offset | Field | Type | Meaning |
| ---: | --- | --- | --- |
| 0 | `power_limit` | `uint8_t` | Raw power limit currently applied on the capacitor side |
| 1 | `chassis_power` | `uint16_t` | Encoded actual chassis power |
| 3 | `referee_power` | `uint16_t` | Encoded total referee output power |
| 5 | `superpower_output_max` | `uint16_t` | Maximum power the capacitor can deliver to side A, a direct power value |
| 7 | `output_capability` | `uint8_t` | Raw output capability, range 0 to 255 |

`chassis_power` and `referee_power` are decoded with `power_w = (encoded - 16384.0f) / 64.0f`. After a new status frame arrives, the decoded result is cached directly, and status frames shorter than 8 bytes are dropped.

Control frame:

| Offset | Field | Type | Value written |
| ---: | --- | --- | --- |
| 0 | `flags` | `uint8_t` | `bit0` set to 1, enabling `enableCONV` |
| 1 | `referee_power_limit` | `uint16_t` | `chassis_ref.rs.chassis_power_limit` |
| 3 | `reserved0` | `uint16_t` | 0 |
| 5 | `reserved1` | `uint8_t` | 0 |
| 6 | `reserved2` | `int16_t` | 0 |

While the Topic delivers no new data, `referee_power_limit` keeps the initial value 0 or the last cached value.

## 3. 在线判定 / Online Detection

模块启动后，在收到第一帧有效状态帧之前处于离线状态。每次收到有效状态帧时，将完整的状态数据与上一帧比较：

- 数据变化时，连续相同帧计数重置为 1。
- 数据相同时，连续相同帧计数加 1。
- 连续相同帧计数达到 200 时判为离线，清空对外状态，保留缓存的裁判功率上限。

After startup the Module is offline until the first valid status frame arrives. Each time a valid status frame arrives, the complete status data is compared with the previous frame:

- When the data changes, the consecutive identical frame count is reset to 1.
- When the data is identical, the count is incremented by 1.
- When the count reaches 200 the Module is judged offline and the exposed state is cleared, keeping the cached referee power limit.

## 4. 构造接口 / Constructor

```cpp
SuperPower(LibXR::CAN& can_bus, const char* chassis_ref_topic_name = "chassis_ref");
```

依赖：

- `can_bus`：`LibXR::CAN`，连接超级电容控制板的 CAN 总线，取自 BSP 的硬件注册（`XR_REGISTER`）。构造时在该总线上注册标准帧过滤器，接收 ID `0x051`。

配置参数：

- `chassis_ref_topic_name`：订阅的裁判系统底盘数据 Topic 名称，默认 `"chassis_ref"`，与 Referee 的 `referee_chassis_tp_name` 相同。构造时查找该 Topic，找不到时触发 `ASSERT`。

Dependencies:

- `can_bus`: the `LibXR::CAN` bus connected to the supercapacitor controller, taken from the BSP's Registration (`XR_REGISTER`). At construction a standard-frame filter for ID `0x051` is registered on the bus.

Configuration parameters:

- `chassis_ref_topic_name`: name of the subscribed referee chassis data Topic, default `"chassis_ref"`, equal to `referee_chassis_tp_name` of Referee. The Topic is looked up at construction and `ASSERT` is triggered when it is missing.

## 5. Topic

| Topic（默认名称） | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `chassis_ref_topic_name`（`chassis_ref`） | 订阅 | `Referee::ChassisPack` | 裁判系统底盘数据，取其中的底盘功率上限 |

| Topic (default name) | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `chassis_ref_topic_name` (`chassis_ref`) | Subscribe | `Referee::ChassisPack` | Referee chassis data, the chassis power limit is taken from it |

## 6. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/SuperPower` 写入的实例，`can_bus` 填写为 BSP 中注册的 CAN 名称：

An instance written by `xrobot instance add QDU-Robomaster/SuperPower`, with `can_bus` set to a CAN name registered by the BSP:

```yaml
modules:
  - module: QDU-Robomaster/SuperPower
    id: super_power
    args:
      - can_bus: can2
      - chassis_ref_topic_name: "chassis_ref"
```

构造时该 Topic 已存在，因此 `referee_chassis_tp_name` 与 `chassis_ref_topic_name` 相同的 `QDU-Robomaster/Referee` 实例列在本实例之前。

The Topic exists at construction, so the `QDU-Robomaster/Referee` instance whose `referee_chassis_tp_name` equals `chassis_ref_topic_name` is listed before this instance.

## 7. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/Referee`：提供 `Referee::ChassisPack` 类型，并创建本模块订阅的底盘数据 Topic。
- LibXR。

硬件：超级电容控制板，经 Classic CAN（标准帧）连接，CAN 对象通过 `XR_REGISTER` 注册。

Dependencies:

- `QDU-Robomaster/Referee`: provides the `Referee::ChassisPack` type and creates the chassis data Topic subscribed by this Module.
- LibXR.

Hardware: the supercapacitor controller, connected over Classic CAN (standard frames), with the CAN object registered with `XR_REGISTER`.
