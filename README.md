# SuperPower

## 1. 模块作用

SuperPower 超级电容电源模块，负责与超电板进行 CAN 双向通讯。

- 接收超电反馈帧（标准帧 ID `0x52`），解析状态字、功率和电容能量。
- 下发控制命令帧（标准帧 ID `0x61`），维持 DCDC 与功率限制策略。
- 订阅 `chassis_ref` 话题，将裁判系统功率上限与缓冲能量写入命令字段。

## 2. 通讯协议与状态字段

反馈数据（`TxDataNew`，8 字节）:

- `status_code`：状态与错误位。
- `chassis_power`：底盘功率（编码值）。
- `referee_power`：裁判功率（编码值）。
- `chassis_power_limit`：底盘功率上限。
- `cap_energy`：电容能量（`0-255`）。

功率解码公式:

- 编码：`encoded = raw * 64 + 16384`
- 解码：`raw = (encoded - 16384) / 64.0f`
- 量程：`-256W ~ +768W`，分辨率：`0.015625W`

状态位定义（`status_code`）:

- bit7：功率级状态（`1` 启动，`0` 未启动）。
- bit6：反馈格式（`1` 新通讯格式，`0` 旧通讯格式）。
- bit1:bit0：错误等级（`NO_ERROR` / `ERROR_RECOVER_AUTO` / `ERROR_RECOVER_MANUAL` / `ERROR_UNRECOVERABLE`）。

命令数据（`CmdData`，8 字节）核心字段:

- `enable_dcdc`：允许启动 DCDC。
- `clear_error`：清除错误。
- `enable_active_charging_limit` / `active_charging_limit_ratio`：主动充电限制。
- `referee_power_limit` / `referee_energy_buffer`：由裁判系统话题更新。

## 3. 主要函数说明

1. `ThreadFunction`
   周期处理线程，每 2ms 执行一次：拉取 `chassis_ref` 话题，更新命令参数并调用 `Update`。

2. `Update`
   处理 CAN 接收队列，计算离线时间；在线时发送命令，离线时清空关键状态。

3. `DecodePower` / `DecodePowerData`
   完成偏移二进制功率解码和反馈帧字段解析。

4. `SendCommand`
   将 `CmdData` 按 8 字节打包，通过 CAN ID `0x61` 下发。

5. `GetCapEnergy` / `IsOnline` / `GetErrorLevel`
   对外提供归一化能量、在线状态、错误等级读取接口。

## 4. 接入步骤

1. 确认硬件中已注册对应 CAN 总线，并可收发标准帧 `0x52/0x61`。
2. 在机器人 YAML 中添加 SuperPower 模块实例，设置 `can_bus_name` 和 `task_stack_depth`。
3. 如需按裁判系统限制功率，确保 Referee 模块已运行并持续发布 `chassis_ref` 话题。
4. 若有功率分配逻辑（如 PowerControl），将 SuperPower 实例注入对应模块。

## 5. 配置示例（YAML）

```yaml
- id: superpower
  name: SuperPower
  constructor_args:
      can_bus_name: can1
      task_stack_depth: 800
      thread_priority: LibXR::Thread::Priority::HIGH
      referee: '@&ref'
```

最小配置可只提供 `can_bus_name` 与 `task_stack_depth`，其余参数使用默认值。

## 6. 依赖与硬件

Required Hardware:

- can

Depends:

- qdu-future/Referee

## 7. 在线判定与失效保护

- 离线阈值为 `1.0s`：超过阈值未收到反馈即判定掉线。
- 掉线后 `online=false`，并将关键反馈量（功率、功率上限、能量、状态字）清零。
- 在线时恢复命令下发，模块可通过 `IsOnline` 和错误等级接口参与上层保护策略。

## 8. 代码入口

- `Modules/SuperPower/SuperPower.hpp`
