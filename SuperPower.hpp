#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 超级电容电源模块
constructor_args:
  - can_bus_name: "can1"
  - task_stack_depth: 2048
template_args: []
required_hardware:
  - can
depends: []
=== END MANIFEST === */
// clang-format on
#include <algorithm>
#include <cstring>

#include "app_framework.hpp"
#include "can.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "libxr_type.hpp"
#include "ramfs.hpp"
#include "thread.hpp"

/* 超电反馈CAN ID (新通讯格式) */
#define SUPERPOWER_FEEDBACK_ID 0x52
/* 发送给超电的命令CAN ID */
#define SUPERPOWER_CMD_ID 0x61

class SuperPower : public LibXR::Application {
 public:
  /* 新通讯格式 (0x52) 反馈数据结构 */
  struct __attribute__((packed)) TxDataNew {
    uint8_t status_code;
    uint16_t chassis_power;
    uint16_t referee_power;
    uint16_t chassis_power_limit;
    uint8_t cap_energy;
  };

  /* 发送给超电的命令数据结构 */
  struct __attribute__((packed)) CmdData {
    uint8_t enable_dcdc : 1;     // 允许启动DCDC
    uint8_t system_restart : 1;  // 系统重启
    uint8_t resv0 : 3;
    uint8_t clear_error : 1;                   // 清除错误
    uint8_t enable_active_charging_limit : 1;  // 启用主动充电限制
    uint8_t use_new_feedback_message : 1;      // 是否使用新的反馈信息格式

    uint16_t referee_power_limit;         // 裁判系统功率限制 W
    uint16_t referee_energy_buffer;       // 裁判系统缓冲能量 J
    uint8_t active_charging_limit_ratio;  // 允许启动DCDC的电容能量比例 (0-255)
    int16_t resv2;
  };

  /* statusCode 位域定义 */
  enum class ErrorLevel : uint8_t {
    NO_ERROR = 0,
    ERROR_RECOVER_AUTO = 1,   /* 可自动恢复的错误 */
    ERROR_RECOVER_MANUAL = 2, /* 需要手动恢复的错误 */
    ERROR_UNRECOVERABLE = 3,  /* 不可恢复的错误，如过流保护触发 */
  };

  SuperPower(
      LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
      const char* can_bus_name, uint32_t task_stack_depth,
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::HIGH)
      : can_(hw.template FindOrExit<LibXR::CAN>({can_bus_name})) {
    UNUSED(app);

    std::memset(&cmd_data_, 0, sizeof(cmd_data_));
    cmd_data_.use_new_feedback_message = 1; /* 默认使用新通讯格式 */
    cmd_data_.enable_dcdc = 1; /* 默认启动DCDC */
    cmd_data_.referee_power_limit = 100; /* 默认裁判系统功率限制 */

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, SuperPower* self, const LibXR::CAN::ClassicPack& pack) {
          RxCallback(in_isr, self, pack);
        },
        this);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, SUPERPOWER_FEEDBACK_ID,
                   SUPERPOWER_FEEDBACK_ID);
    thread_.Create(this, ThreadFunction, "SuperPowerThread", task_stack_depth,
                   LibXR::Thread::Priority::HIGH);
  }

  static void ThreadFunction(SuperPower* super_power) {
    while (true) {
      super_power->Update();
      super_power->thread_.Sleep(2);
    }
  }

  void Update() {
    LibXR::CAN::ClassicPack pack;
    if (recv_.Pop(pack) == LibXR::ErrorCode::OK) {
      this->DecodePowerData(pack);
    }

    auto now = LibXR::Timebase::GetMilliseconds();
    if (last_rx_time_ms_ > now) {
      now = last_rx_time_ms_;
    }
    float time_since_last_rx = (now - last_rx_time_ms_).ToSecondf();

    dt_ = time_since_last_rx;

    /*如果0.1s没有收到新的数据 认为超电掉线*/
    if (time_since_last_rx > 0.1f) {
      online_ = false;
      chassis_power_ = 0.0f;
      referee_power_ = 0.0f;
      chassis_power_limit_ = 0.0f;
      cap_energy_ = 0;
      status_code_ = 0;
    } else {
      online_ = true;
    }

    /* 发送命令给超电 */
    SendCommand();
  }

  static void RxCallback(bool in_isr, SuperPower* self,
                         const LibXR::CAN::ClassicPack& pack) {
    UNUSED(in_isr);
    if (pack.id == SUPERPOWER_FEEDBACK_ID) {
      self->last_rx_time_ms_ = LibXR::Timebase::GetMilliseconds();
      self->PushToQueue(pack);
    }
  }

  void PushToQueue(const LibXR::CAN::ClassicPack& pack) { recv_.Push(pack); }

  /**
   * @brief 偏移二进制解码: encoded = raw * 64 + 16384
   *        解码: raw = (encoded - 16384) / 64.0f
   *        量程 -256W ~ +768W, 分辨率 0.015625W
   */
  static float DecodePower(uint16_t encoded) {
    return (static_cast<float>(encoded) - 16384.0f) / 64.0f;
  }

  void DecodePowerData(const LibXR::CAN::ClassicPack& pack) {
    TxDataNew data;
    std::memcpy(&data, pack.data, sizeof(TxDataNew));
    status_code_ = data.status_code;
    chassis_power_ = DecodePower(data.chassis_power);
    referee_power_ = DecodePower(data.referee_power);
    chassis_power_limit_ = static_cast<float>(data.chassis_power_limit);
    cap_energy_ = data.cap_energy;
  }

  float GetChassisPower() { return this->chassis_power_; }

  float GetRefereePower() { return this->referee_power_; }

  float GetChassisPowerLimit() { return this->chassis_power_limit_; }

  uint8_t GetCapEnergy() { return this->cap_energy_; }

  uint8_t GetStatusCode() { return this->status_code_; }

  /* statusCode 位域解析 */

  /*功率级状态 1为启动 0为未启动（触发保护或主控板禁用）*/
  bool IsPowerStageOn() { return (status_code_ >> 7) & 0x01; }

  /* 反馈信息格式 | 1为新通讯格式，0为旧通讯格式（RM2024）*/
  bool IsNewFeedbackFormat() { return (status_code_ >> 6) & 0x01; }

  /*错误等级*/
  ErrorLevel GetErrorLevel() {
    return static_cast<ErrorLevel>(status_code_ & 0x03);
  }

  bool IsOnline() { return online_; }

  void OnMonitor() override {}

 private:
  /**
   * @brief 通过CAN发送命令给超电
   */
  void SendCommand() {
    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = SUPERPOWER_CMD_ID;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = sizeof(CmdData);
    static_assert(sizeof(CmdData) == 8, "CmdData must be 8 bytes for CAN");
    std::memcpy(tx_pack.data, &cmd_data_, sizeof(CmdData));
    can_->AddMessage(tx_pack);
  }

  LibXR::Thread thread_;
  float chassis_power_ = 0.0f;
  float referee_power_ = 0.0f;
  float chassis_power_limit_ = 0.0f;
  uint8_t cap_energy_ = 0;
  uint8_t status_code_ = 0;
  LibXR::CAN* can_;

  CmdData cmd_data_{}; /* 发送给超电的命令数据 */

  LibXR::MillisecondTimestamp last_rx_time_ms_ = 0.0f;
  float dt_ = 0.0f;
  bool online_ = false;

  LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv_{1};
};
