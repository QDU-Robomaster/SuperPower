#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 超级电容电源模块
constructor_args:
  - can_bus_name: "can1"
template_args: []
required_hardware:
  - can
depends:
  - qdu-future/Referee
=== END MANIFEST === */
// clang-format on
#include <cstring>

#include "Referee.hpp"
#include "app_framework.hpp"
#include "can.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "lockfree_queue.hpp"
#include "message.hpp"
#include "timer.hpp"

/* 超电状态帧 CAN 标准 ID 超电发给主控 */
#define FEEDBACK_ID 0x51
/* 超电控制帧 CAN 标准 ID 主控发给超电 */
#define COMMAND_ID 0x61

/**
 * @class SuperPower
 * @brief 主控侧超级电容通信模块
 * @details 接收超电状态帧 同步裁判系统功率上限 定时下发控制帧
 *          给上层功率控制提供在线状态和功率数据
 */
class SuperPower : public LibXR::Application {
 public:
  /**
   * @brief 超电状态帧
   * @details 对应标准帧 ID 0x51 数据长度固定为 8 字节
   */
  struct __attribute__((packed)) StatusData {
    uint8_t power_limit;            /* 超电侧当前功率限制 */
    uint16_t chassis_power;         /* 底盘实际功率编码值 */
    uint16_t referee_power;         /* 裁判系统总输出功率编码值 */
    uint16_t superpower_output_max; /* 超电可向 A 侧输出的最大功率 */
    uint8_t output_capability;      /* 输出能力原始值，范围 0~255 */
  };

  /**
   * @brief 超电控制帧
   * @details 对应标准帧 ID 0x61 数据长度固定为 8 字节
   */
  struct __attribute__((packed)) CommandData {
    uint8_t flags;                /* bit0 为 enableCONV 使能位 */
    uint16_t referee_power_limit; /* 下发给超电的裁判功率上限 */
    uint16_t reserved0;           /* 协议保留字段，发送 0 */
    uint8_t reserved1;            /* 协议保留字段，发送 0 */
    int16_t reserved2;            /* 协议保留字段，发送 0 */
  };

  /**
   * @brief SuperPower 构造函数
   * @param hw 硬件容器引用
   * @param app 应用管理器引用
   * @param can_bus_name CAN 总线名称
   * @details 构造时注册 0x51 状态帧接收过滤器 订阅 chassis_ref 话题
   *          再启动定时任务维护在线状态和控制帧发送
   */
  SuperPower(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
             const char* can_bus_name)
      : can_(hw.template FindOrExit<LibXR::CAN>({can_bus_name})) {
    UNUSED(app);

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, SuperPower* self, const LibXR::CAN::ClassicPack& pack) {
          RxCallback(in_isr, self, pack);
        },
        this);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, FEEDBACK_ID, FEEDBACK_ID);

    RegisterRefereeCallback();
    timer_handle_ = LibXR::Timer::CreateTask(TimerTask, this, UPDATE_PERIOD_MS);
    LibXR::Timer::Add(timer_handle_);
    LibXR::Timer::Start(timer_handle_);
  }

  /**
   * @brief 订阅裁判系统底盘数据
   * @details 从 chassis_ref 话题获取裁判系统底盘功率上限
   *          后面发控制帧时写到 referee_power_limit 字段里
   */
  void RegisterRefereeCallback() {
    auto topic_handle = LibXR::Topic::Find("chassis_ref", nullptr);
    ASSERT(topic_handle != nullptr);

    auto referee_callback = LibXR::Topic::Callback::Create(
        [](bool in_isr, SuperPower* self,
           const Referee::ChassisPack& chassis_pack) {
          UNUSED(in_isr);
          self->referee_power_limit_ = chassis_pack.rs.chassis_power_limit;
        },
        this);

    LibXR::Topic chassis_ref_topic(topic_handle);
    chassis_ref_topic.RegisterCallback(referee_callback);
  }

  /**
   * @brief 处理超电状态帧
   * @param pack 接收到的 CAN 标准帧
   * @details 只处理长度不小于 StatusData 的状态帧
   *          解析完成后刷新接收时间并标记已经收到状态帧
   */
  void OnFeedbackFrame(const LibXR::CAN::ClassicPack& pack) {
    if (pack.dlc < sizeof(StatusData)) {
      return;
    }

    DecodeStatusData(pack);

    const uint32_t NOW_MS =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    last_rx_time_ms_ = NOW_MS;
    status_received_ = true;
  }

  /**
   * @brief 解析超电状态帧
   * @param pack 接收到的 CAN 标准帧
   * @details 协议字段按 packed 结构体直接拷贝
   *          功率字段按各自协议语义处理后再缓存
   */
  void DecodeStatusData(const LibXR::CAN::ClassicPack& pack) {
    StatusData data{};
    std::memcpy(&data, pack.data, sizeof(StatusData));
    power_limit_ = data.power_limit;
    chassis_power_ = FilterChassisPower(DecodeOffsetPower(data.chassis_power));
    referee_power_ = FilterRefereePower(DecodeOffsetPower(data.referee_power));
    superpower_output_max_ = DecodeDirectPower(data.superpower_output_max);
    output_capability_ = data.output_capability;
  }

  /**
   * @brief 获取底盘实际功率
   * @return 解码后的底盘实际功率，单位 W，离线时返回 0
   */
  float GetChassisPower() {
    if (!RefreshOnlineState()) {
      return 0.0f;
    }

    return chassis_power_;
  }

  /**
   * @brief 获取归一化后的超电输出能力
   * @details 这个接口为了兼容旧上层命名保留
   *          实际含义是输出能力比例 不是电容容量或剩余电量
   * @return output_capability / 255.0f 离线时返回 0
   */
  float GetCapEnergy() {
    RefreshOnlineState();
    return static_cast<float>(output_capability_) / 255.0f;
  }

  /**
   * @brief 获取裁判系统总输出功率
   * @return 解码后的裁判系统总输出功率，单位 W，离线时返回 0
   */
  float GetRefereePower() {
    if (!RefreshOnlineState()) {
      return 0.0f;
    }

    return referee_power_;
  }

  /**
   * @brief 获取超电可向 A 侧输出的最大功率
   * @return 当前最大输出功率，单位 W，离线时返回 0
   */
  float GetSuperPowerOutputMax() {
    if (!RefreshOnlineState()) {
      return 0.0f;
    }

    return superpower_output_max_;
  }

  /**
   * @brief 获取超电认为的当前功率限制
   * @return 功率限制原始值，离线时返回 0
   */
  uint8_t GetPowerLimit() {
    RefreshOnlineState();
    return power_limit_;
  }

  /**
   * @brief 获取超电输出能力原始字节值
   * @return 输出能力原始值，范围 0~255，离线时返回 0
   */
  uint8_t GetOutputCapabilityRaw() {
    RefreshOnlineState();
    return output_capability_;
  }

  /**
   * @brief 获取超电在线状态
   * @return true 表示在线 false 表示离线或还没收到状态帧
   */
  bool IsOnline() { return RefreshOnlineState(); }

  /**
   * @brief 监控回调
   */
  void OnMonitor() override {}

 private:
  /* 控制帧 flags 的 bit0 对应协议里的 enableCONV 使能位 */
  static constexpr uint8_t ENABLE_CONV_MASK = 0x01;
  /* 在线检测和控制帧发送调度周期 */
  static constexpr uint32_t UPDATE_PERIOD_MS = 2;
  /* 超电控制帧最小发送间隔 */
  static constexpr uint32_t COMMAND_PERIOD_MS = 5;
  /* 超过这个时间还没收到状态帧就认为超电离线 */
  static constexpr float OFFLINE_TIMEOUT_S = 1.0f;
  /* 功率字段零点偏移 */
  static constexpr float POWER_ENCODE_OFFSET = 16384.0f;
  /* 功率字段缩放倍数 */
  static constexpr float POWER_ENCODE_SCALE = 64.0f;
  /* 功率低通滤波系数 */
  static constexpr float POWER_FILTER_ALPHA = 0.05f;

  /**
   * @brief 解码带零点偏移的功率字段
   * @param encoded 协议里的功率编码值
   * @return 解码后的功率，单位 W
   */
  static float DecodeOffsetPower(uint16_t encoded) {
    return (static_cast<float>(encoded) - POWER_ENCODE_OFFSET) /
           POWER_ENCODE_SCALE;
  }

  /**
   * @brief 解码直接功率字段
   * @param power 协议里的功率值
   * @return 功率值，单位 W
   */
  static float DecodeDirectPower(uint16_t power) {
    return static_cast<float>(power);
  }

  /**
   * @brief 滤波底盘实际功率
   * @param power 解码后的底盘实际功率
   * @return 低通滤波后的底盘实际功率
   */
  float FilterChassisPower(float power) {
    if (!chassis_power_filter_ready_) {
      chassis_power_filter_ready_ = true;
      return power;
    }

    return chassis_power_ + POWER_FILTER_ALPHA * (power - chassis_power_);
  }

  /**
   * @brief 滤波裁判系统总输出功率
   * @param power 解码后的裁判系统总输出功率
   * @return 低通滤波后的裁判系统总输出功率
   */
  float FilterRefereePower(float power) {
    if (!referee_power_filter_ready_) {
      referee_power_filter_ready_ = true;
      return power;
    }

    return referee_power_ + POWER_FILTER_ALPHA * (power - referee_power_);
  }

  /**
   * @brief 定时处理在线状态和控制帧下发
   * @param self SuperPower 实例指针
   */
  static void TimerTask(SuperPower* self) { self->Update(); }

  /**
   * @brief CAN 接收回调
   * @details 回调里只缓存最新状态帧 实际解析放到定时任务里做
   */
  static void RxCallback(bool in_isr, SuperPower* self,
                         const LibXR::CAN::ClassicPack& pack) {
    UNUSED(in_isr);
    while (self->recv_queue_.Push(pack) != LibXR::ErrorCode::OK) {
      self->recv_queue_.Pop();
    }
  }

  /**
   * @brief 定时刷新模块状态
   * @details 在线时按周期下发控制帧 离线或还没收到状态帧时不发送
   */
  void Update() {
    LibXR::CAN::ClassicPack pack;
    while (recv_queue_.Pop(pack) == LibXR::ErrorCode::OK) {
      OnFeedbackFrame(pack);
    }

    if (!RefreshOnlineState()) {
      return;
    }

    const uint32_t NOW_MS =
        static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
    SendCommandFrame(NOW_MS);
  }

  /**
   * @brief 超过离线阈值后清空对外状态
   * @details 只清空状态帧更新出来的数据 不清空裁判系统功率上限
   */
  void ClearStatus() {
    power_limit_ = 0;
    chassis_power_ = 0.0f;
    chassis_power_filter_ready_ = false;
    referee_power_ = 0.0f;
    referee_power_filter_ready_ = false;
    superpower_output_max_ = 0.0f;
    output_capability_ = 0;
  }

  /**
   * @brief 按最后接收时间刷新在线状态
   * @return true 表示状态帧没超时 false 表示还没收到或已经超时
   */
  bool RefreshOnlineState() {
    if (!status_received_) {
      ClearStatus();
      return false;
    }

    const uint32_t LAST_RX_MS = last_rx_time_ms_;
    const auto NOW_MS = LibXR::Timebase::GetMilliseconds();
    const auto LAST_RX_TIMESTAMP = LibXR::MillisecondTimestamp(LAST_RX_MS);
    const float TIME_SINCE_LAST_RX = (NOW_MS - LAST_RX_TIMESTAMP).ToSecondf();

    if (TIME_SINCE_LAST_RX > OFFLINE_TIMEOUT_S) {
      if (last_rx_time_ms_ == LAST_RX_MS) {
        ClearStatus();
      }
      return false;
    }

    return true;
  }

  /**
   * @brief 按 5ms 节流下发控制帧
   * @param now_ms 当前毫秒时间戳
   */
  void SendCommandFrame(uint32_t now_ms) {
    const uint32_t LAST_TX_MS = last_command_tx_time_ms_;
    const auto NOW_TIMESTAMP = LibXR::MillisecondTimestamp(now_ms);
    const auto LAST_TX_TIMESTAMP = LibXR::MillisecondTimestamp(LAST_TX_MS);

    if ((NOW_TIMESTAMP - LAST_TX_TIMESTAMP).ToMillisecond() <
        COMMAND_PERIOD_MS) {
      return;
    }

    SendCommandFrame();
    last_command_tx_time_ms_ = now_ms;
  }

  /**
   * @brief 发送超电控制帧
   * @details 当前固定打开 enableCONV
   *          并把最新裁判系统底盘功率上限写到 referee_power_limit 字段
   */
  void SendCommandFrame() {
    CommandData command_data{};
    command_data.flags = ENABLE_CONV_MASK;
    command_data.referee_power_limit = referee_power_limit_;

    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = COMMAND_ID;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = sizeof(CommandData);
    static_assert(sizeof(CommandData) == 8,
                  "CommandData must be 8 bytes for CAN");
    std::memcpy(tx_pack.data, &command_data, sizeof(CommandData));
    can_->AddMessage(tx_pack);
  }

  LibXR::CAN* can_;
  LibXR::Timer::TimerHandle timer_handle_ = nullptr;
  LibXR::LockFreeQueue<LibXR::CAN::ClassicPack> recv_queue_{1};

  uint16_t referee_power_limit_ = 0;        /* 裁判系统功率上限 */
  float chassis_power_ = 0.0f;              /* 解码后的底盘实际功率，单位 W */
  float referee_power_ = 0.0f;              /* 解码后的裁判系统总功率，单位 W */
  float superpower_output_max_ = 0.0f;      /* 最大输出功率，单位 W */
  uint32_t last_rx_time_ms_ = 0;            /* 最后接收时间 */
  uint32_t last_command_tx_time_ms_ = 0;    /* 最后发送时间 */
  uint8_t power_limit_ = 0;                 /* 功率限制原始值 */
  uint8_t output_capability_ = 0;           /* 输出能力原始值 */
  bool status_received_ = false;            /* 是否收到过状态帧 */
  bool chassis_power_filter_ready_ = false; /* 底盘功率滤波是否初始化 */
  bool referee_power_filter_ready_ = false; /* 裁判功率滤波是否初始化 */
};
