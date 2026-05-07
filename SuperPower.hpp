#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 超级电容电源模块
constructor_args:
  - can_bus_name: "can1"
  - task_stack_depth: 800
  - thread_priority: LibXR::Thread::Priority::HIGH
  - referee: "@nullptr"
template_args: []
required_hardware:
  - can
depends:
  - qdu-future/Referee
=== END MANIFEST === */
// clang-format on
#include <algorithm>
#include <cstdio>
#include <cstring>

#include "Referee.hpp"
#include "app_framework.hpp"
#include "can.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "libxr_type.hpp"
#include "message.hpp"
#include "ramfs.hpp"
#include "thread.hpp"

#define FEEDBACK_ID 0x051
#define COMMAND_ID 0x061

class SuperPower : public LibXR::Application {
 public:
  /**
   * @brief 超电状态帧
   * @details 对应标准帧 ID 0x051
   */
  struct __attribute__((packed)) StatusData {
    uint8_t power_limit;
    uint16_t chassis_power;
    uint16_t referee_power;
    uint16_t supercap_output_max;
    uint8_t output_capability;
  };

  /**
   * @brief 超电控制帧
   * @details 对应标准帧 ID 0x061
   */
  struct __attribute__((packed)) CommandData {
    uint8_t flags;
    uint16_t referee_power_limit;
    uint16_t reserved0;
    uint8_t reserved1;
    int16_t reserved2;
  };

  /**
   * @brief SuperPower 构造函数
   * @param hw 硬件容器引用
   * @param app 应用管理器引用
   * @param can_bus_name CAN 总线名
   * @param task_stack_depth 线程栈深度
   * @param thread_priority 线程优先级
   * @param referee 裁判系统模块指针，当前仅为兼容构造接口保留
   */
  SuperPower(
      LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
      const char* can_bus_name, uint32_t task_stack_depth,
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::HIGH,
      Referee* referee = nullptr)
      : can_(hw.template FindOrExit<LibXR::CAN>({can_bus_name})) {
    UNUSED(app);
    UNUSED(referee);

    LibXR::Memory::FastSet(&command_data_, 0, sizeof(command_data_));
    command_data_.flags = ENABLE_CONV_MASK;

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, SuperPower* self, const LibXR::CAN::ClassicPack& pack) {
          UNUSED(in_isr);
          if (pack.id == FEEDBACK_ID) {
            self->last_rx_time_ms_ = LibXR::Timebase::GetMilliseconds();
            self->recv_.Push(pack);
          }
        },
        this);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, FEEDBACK_ID,
                   FEEDBACK_ID);

    thread_.Create(this, ThreadFunction, "SuperPowerThread", task_stack_depth,
                   thread_priority);
  }

  /**
   * @brief SuperPower 后台线程
   * @param super_power SuperPower 实例指针
   */
  static void ThreadFunction(SuperPower* super_power) {
    LibXR::Topic::ASyncSubscriber<Referee::ChassisPack> referee_suber(
        "chassis_ref");
    referee_suber.StartWaiting();

    auto last_time = LibXR::Timebase::GetMilliseconds();

    while (true) {
      if (referee_suber.Available()) {
        const auto chassis_pack = referee_suber.GetData();
        super_power->command_data_.referee_power_limit =
            chassis_pack.rs.chassis_power_limit;
        super_power->referee_chassis_pack_ = chassis_pack;
        referee_suber.StartWaiting();
      }
      super_power->Update();

      super_power->thread_.SleepUntil(last_time, 2);
    }
  }

  /**
   * @brief 更新超电通信状态
   * @details 处理接收队列、在线判定以及 5ms 周期控制帧发送
   */
  void Update() {
    constexpr float OFFLINE_TIMEOUT_S = 1.0f;
    constexpr uint32_t COMMAND_PERIOD_MS = 5;

    LibXR::CAN::ClassicPack pack;
    if (recv_.Pop(pack) == LibXR::ErrorCode::OK) {
      DecodeStatusData(pack);
    }

    auto now = LibXR::Timebase::GetMilliseconds();
    if (last_rx_time_ms_ > now) {
      now = last_rx_time_ms_;
    }
    const float TIME_SINCE_LAST_RX = (now - last_rx_time_ms_).ToSecondf();
    dt_ = TIME_SINCE_LAST_RX;

    /* 超过阈值未收到反馈，认为超电掉线并清空关键状态。 */
    if (TIME_SINCE_LAST_RX > OFFLINE_TIMEOUT_S) {
      online_ = false;
      chassis_power_ = 0.0f;
      referee_power_ = 0.0f;
      power_limit_ = 0;
      supercap_output_max_ = 0.0f;
      output_capability_ = 0;
    } else {
      online_ = true;
      const bool SHOULD_SEND_COMMAND =
          (now - last_command_tx_time_ms_).ToSecondf() * 1000.0f >=
          static_cast<float>(COMMAND_PERIOD_MS);
      if (SHOULD_SEND_COMMAND) {
        SendCommandFrame();
        last_command_tx_time_ms_ = now;
      }
    }
  }

  /**
   * @brief 解析超电状态帧
   * @param pack 接收到的 CAN 标准帧
   */
  void DecodeStatusData(const LibXR::CAN::ClassicPack& pack) {
    StatusData data{};
    std::memcpy(&data, pack.data, sizeof(StatusData));
    power_limit_ = data.power_limit;
    chassis_power_ = static_cast<float>(data.chassis_power);
    referee_power_ = static_cast<float>(data.referee_power);
    supercap_output_max_ = static_cast<float>(data.supercap_output_max);
    output_capability_ = data.output_capability;
  }

  /**
   * @brief 获取底盘实际功率
   * @return 底盘实际功率
   */
  float GetChassisPower() { return chassis_power_; }

  /**
   * @brief 获取归一化后的超电输出能力
   * @details 该接口为兼容旧上层命名保留，语义并非电容容量
   * @return `output_capability / 255.0f`
   */
  float GetCapEnergy() {
    return static_cast<float>(output_capability_) / 255.0f;
  }

  /**
   * @brief 获取裁判系统总输出功率
   * @return 裁判系统总输出功率
   */
  float GetRefereePower() { return referee_power_; }

  /**
   * @brief 获取超电可向 A 侧输出的最大功率
   * @return 当前最大输出功率
   */
  float GetSuperCapOutputMax() { return supercap_output_max_; }

  /**
   * @brief 获取超电认为的当前功率限制
   * @return 功率限制原始值
   */
  uint8_t GetPowerLimit() { return power_limit_; }

  /**
   * @brief 获取超电输出能力原始字节值
   * @return 输出能力原始值
   */
  uint8_t GetOutputCapabilityRaw() { return output_capability_; }

  /**
   * @brief 获取超电在线状态
   * @return `true` 表示在线，`false` 表示离线
   */
  bool IsOnline() { return online_; }

  /**
   * @brief 监控回调
   */
  void OnMonitor() override {}

 private:
  /* 控制帧 flags 的 bit0，对应协议中的 enableCONV 使能位。 */
  static constexpr uint8_t ENABLE_CONV_MASK = 0x01;

  /**
   * @brief 发送超电控制帧
   */
  void SendCommandFrame() {
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = COMMAND_ID;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = sizeof(CommandData);
    static_assert(sizeof(CommandData) == 8,
                  "CommandData must be 8 bytes for CAN");
    std::memcpy(tx_pack.data, &command_data_, sizeof(CommandData));
    can_->AddMessage(tx_pack);
  }
  LibXR::Thread thread_;

  float chassis_power_ = 0.0f;
  float referee_power_ = 0.0f;
  float supercap_output_max_ = 0.0f;
  uint8_t power_limit_ = 0;
  uint8_t output_capability_ = 0;

  LibXR::CAN* can_;
  CommandData command_data_{};
  Referee::ChassisPack referee_chassis_pack_{};

  LibXR::MillisecondTimestamp last_rx_time_ms_ = 0.0f;
  LibXR::MillisecondTimestamp last_command_tx_time_ms_ = 0.0f;
  float dt_ = 0.0f;
  bool online_ = false;

  LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv_{1};
};
