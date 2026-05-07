# SuperPower

## 1. 模块作用

`SuperPower` 是主控侧的超电通信模块，用于在机器人主控与外部超级电容控制板之间建立 CAN 双向通讯。

本仓库中的实现是**主控侧驱动**，不是超电板固件本体，因此通信方向按主控视角解释：

- 接收超电状态帧：标准帧 ID `0x051`
- 发送超电控制帧：标准帧 ID `0x061`
- 使用 `Classic CAN`、标准帧、`8` 字节数据段
- 订阅 `chassis_ref` 话题，将裁判系统功率上限写入控制帧

当前实现来源于你给出的超电 README 通讯定义，但为了适配本 BSP 的模块职责，线协议格式保持一致，收发方向按主控侧重新解释。

## 2. 通讯协议

### 2.1 接收状态帧：超电 -> 主控（ID = `0x051`）

代码中的状态帧结构如下：

```c
struct __attribute__((packed)) StatusData {
    uint8_t power_limit;
    uint16_t chassis_power;
    uint16_t referee_power;
    uint16_t supercap_output_max;
    uint8_t output_capability;
};
```

字段说明：

| 字段 | 字节数 | 含义 |
|---|---:|---|
| `power_limit` | 1 | 超电认为的当前功率限制 |
| `chassis_power` | 2 | 底盘实际功率 |
| `referee_power` | 2 | 裁判系统总输出功率 |
| `supercap_output_max` | 2 | 超电当前可向 A 侧输出的最大功率 |
| `output_capability` | 1 | 当前输出能力百分比原始值 |

实现约定：

- 8 字节按 `packed` 结构体内存布局直接 `memcpy`
- 多字节字段按 STM32 本地小端格式解释
- `chassis_power`、`referee_power`、`supercap_output_max` 当前直接按 `uint16_t` 原始值读取，不做额外缩放
- `GetCapEnergy()` 返回 `output_capability / 255.0f`，用于兼容现有 `PowerControl` / `Chassis` 上层逻辑

注意：

- `GetCapEnergy()` 的名字来自旧接口历史，当前语义是“归一化后的输出能力”，不是“电容容量”

### 2.2 发送控制帧：主控 -> 超电（ID = `0x061`）

代码中的控制帧结构如下：

```c
struct __attribute__((packed)) CommandData {
    uint8_t flags;
    uint16_t referee_power_limit;
    uint16_t reserved0;
    uint8_t reserved1;
    int16_t reserved2;
};
```

当前实际使用字段：

| 字段 | 字节数 | 含义 | 来源 |
|---|---:|---|---|
| `flags bit0` | 1 bit | `enableCONV`，是否允许变换器工作 | 当前实现固定为 `1` |
| `referee_power_limit` | 2 | 主控下发给超电的裁判功率限制 | `chassis_ref.rs.chassis_power_limit` |
| `reserved0` | 2 | 保留 | 固定发送 `0` |
| `reserved1` | 1 | 保留 | 固定发送 `0` |
| `reserved2` | 2 | 保留 | 固定发送 `0` |

说明：

- 当前实现未使用 C 位域结构直接映射 `enableCONV`，而是使用 `flags` 字节的 `bit0`
- 这和你提供的协议语义一致，只是代码实现方式更直接

## 3. 运行行为

模块运行逻辑如下：

1. 构造时在指定 CAN 总线上注册接收过滤器，仅接收标准帧 `0x051`
2. 创建后台线程，线程周期为 `2 ms`
3. 若 `chassis_ref` 有新数据，则更新 `referee_power_limit`
4. 超电在线时，每 `5 ms` 发送一次 `0x061` 控制帧
5. 超过 `1.0 s` 未收到新的 `0x051` 状态帧，则判定超电离线
6. 离线后清空关键状态量，并将 `online` 置为 `false`

说明：

- 你给出的原始超电说明中，`0x051` 的发送节拍来自 `TIM2_IRQHandler`
- 在本 BSP 中，`SuperPower` 是应用层模块，因此改为线程内软件定时发送，行为等价但不依赖中断实现

## 4. 对外接口

当前 `SuperPower` 只保留以下对外接口：

- `GetChassisPower()`：返回 `chassis_power`
- `GetRefereePower()`：返回 `referee_power`
- `GetSuperCapOutputMax()`：返回 `supercap_output_max`
- `GetPowerLimit()`：返回 `power_limit`
- `GetCapEnergy()`：返回 `output_capability / 255.0f`
- `GetOutputCapabilityRaw()`：返回 `output_capability` 原始字节
- `IsOnline()`：返回在线状态

旧协议遗留接口已移除，不再保留状态字、错误等级或反馈格式相关 API。

## 5. 配置示例

```yaml
- id: superpower
  name: SuperPower
  constructor_args:
    can_bus_name: can1
    task_stack_depth: 800
    thread_priority: LibXR::Thread::Priority::HIGH
    referee: '@&ref'
```

最小使用要求：

- `can_bus_name` 必须对应 `User/app_main.cpp` 中已注册的 CAN 设备
- 若希望自动同步裁判功率上限，需要系统中已有 `Referee` 模块并持续发布 `chassis_ref`

## 6. 依赖与硬件

Required Hardware:

- can

Depends:

- qdu-future/Referee

## 7. 代码入口

- `Modules/SuperPower/SuperPower.hpp`
