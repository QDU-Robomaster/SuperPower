#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 超级电容 CAN 通信模块：接收超电状态帧、同步裁判系统功率上限并回发控制帧 / Supercapacitor CAN communication Module that receives the capacitor status frames, synchronizes the referee power limit and sends the control frames
depends:
- id: QDU-Robomaster/Referee
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on
#include <cstring>
#include <memory>

#include "Referee.hpp"
#include "can.hpp"
#include "libxr_def.hpp"
#include "libxr_mem.hpp"
#include "libxr_time.hpp"
#include "message.hpp"

/// 超电状态帧的 CAN 标准 ID，超电发给主控
/// CAN standard ID of the capacitor status frame, capacitor to main controller
#define FEEDBACK_ID 0x51
/// 超电控制帧的 CAN 标准 ID，主控发给超电
/// CAN standard ID of the capacitor control frame, main controller to capacitor
#define COMMAND_ID 0x61

/**
 * @brief 超级电容 CAN 通信模块。
 *        Supercapacitor CAN communication Module.
 *
 * @details 接收超电状态帧，同步裁判系统功率上限，在 CAN 接收回调中发送控制帧，
 *          并向上层功率控制提供在线状态与功率数据。
 *          Receives the capacitor status frames, synchronizes the referee power limit,
 *          sends the control frames from the CAN receive callback and provides the online
 *          state and power data to the power control.
 */
class SuperPower
{
 public:
  /**
   * @brief 超电状态帧，标准帧 ID 0x51，数据长度 8 字节。
   *        Capacitor status frame, standard ID 0x51, 8 bytes of data.
   */
  struct __attribute__((packed)) StatusData
  {
    uint8_t power_limit;  ///< 超电侧当前功率限制原始值
    ///< Raw power limit currently applied on the capacitor side
    uint16_t chassis_power;  ///< 底盘实际功率编码值
    ///< Encoded actual chassis power
    uint16_t referee_power;  ///< 裁判系统总输出功率编码值
    ///< Encoded total referee output power
    uint16_t superpower_output_max;  ///< 超电可向 A 侧输出的最大功率，单位 W
    ///< Maximum power the capacitor can deliver to side A, in W
    uint8_t output_capability;  ///< 输出能力原始值，范围 0 到 255
    ///< Raw output capability, range 0 to 255
  };

  /**
   * @brief 超电控制帧，标准帧 ID 0x61，数据长度 8 字节。
   *        Capacitor control frame, standard ID 0x61, 8 bytes of data.
   */
  struct __attribute__((packed)) CommandData
  {
    uint8_t flags;  ///< `bit0` 为 enableCONV 使能位
    ///< `bit0` is the enableCONV enable bit
    uint16_t referee_power_limit;  ///< 下发给超电的裁判系统功率上限，单位 W
    ///< Referee power limit sent to the capacitor, in W
    uint16_t reserved0;  ///< 保留字段，发送 0
    ///< Reserved field, 0 is sent
    uint8_t reserved1;  ///< 保留字段，发送 0
    ///< Reserved field, 0 is sent
    int16_t reserved2;  ///< 保留字段，发送 0
    ///< Reserved field, 0 is sent
  };

  /**
   * @brief 构造 SuperPower，注册 0x51 状态帧接收过滤器并订阅裁判系统底盘数据 Topic。
   *        Construct SuperPower, register the receive filter for the 0x51 status frame
   *        and subscribe to the referee chassis data Topic.
   *
   * @param can_bus 连接超级电容控制板的 CAN 总线。
   *                CAN bus connected to the supercapacitor controller.
   * @param chassis_ref_topic_name 订阅的裁判系统底盘数据 Topic 名称。
   *                               Name of the subscribed referee chassis data Topic.
   */
  SuperPower(
      LibXR::CAN& can_bus,
      const char* chassis_ref_topic_name = "chassis_ref")
      : can_(std::addressof(can_bus))
  {
    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, SuperPower* self, const LibXR::CAN::ClassicPack& pack)
        { RxCallback(in_isr, self, pack); }, this);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, FEEDBACK_ID, FEEDBACK_ID);

    RegisterRefereeCallback(chassis_ref_topic_name);
  }

  /**
   * @brief 订阅裁判系统底盘数据 Topic，缓存底盘功率上限供控制帧使用；找不到 Topic 时触发
   *        `ASSERT`。
   *        Subscribe to the referee chassis data Topic and cache the chassis power limit
   *        for the control frame; `ASSERT` is triggered when the Topic is missing.
   *
   * @param topic_name 裁判系统底盘数据 Topic 名称。
   *                   Name of the referee chassis data Topic.
   */
  void RegisterRefereeCallback(const char* topic_name)
  {
    auto topic_handle = LibXR::Topic::Find(topic_name, nullptr);
    ASSERT(topic_handle != nullptr);

    auto referee_callback = LibXR::Topic::Callback::Create(
        [](bool in_isr, SuperPower* self, const Referee::ChassisPack& chassis_pack)
        {
          UNUSED(in_isr);
          self->referee_power_limit_ = chassis_pack.rs.chassis_power_limit;
        },
        this);

    LibXR::Topic chassis_ref_topic(topic_handle);
    chassis_ref_topic.RegisterCallback(referee_callback);
  }

  /**
   * @brief 处理超电状态帧：长度不足的帧被丢弃，其余帧更新连续相同帧计数、
   *        解码并在在线时发送控制帧。
   *        Handle a capacitor status frame: frames that are too short are dropped, other
   *        frames update the identical-frame count, are decoded and trigger a control
   *        frame while online.
   *
   * @param pack 接收到的 CAN 标准帧。
   *             Received CAN standard frame.
   */
  void OnFeedbackFrame(const LibXR::CAN::ClassicPack& pack)
  {
    if (pack.dlc < sizeof(StatusData))
    {
      return;
    }

    StatusData data{};
    LibXR::Memory::FastCopy(&data, pack.data, sizeof(StatusData));
    UpdateSameFrameCount(data);
    DecodeStatusData(data);
    status_received_ = true;

    if (RefreshOnlineState())
    {
      const uint32_t NOW_MS = static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());
      SendCommandFrame(NOW_MS);
    }
  }

  /**
   * @brief 解码状态帧并缓存功率、功率限制与输出能力。
   *        Decode a status frame and cache the power values, the power limit and the
   *        output capability.
   *
   * @param data 从 CAN 数据区拷贝出的状态帧。
   *             Status frame copied from the CAN data.
   */
  void DecodeStatusData(const StatusData& data)
  {
    power_limit_ = data.power_limit;
    chassis_power_ = DecodeOffsetPower(data.chassis_power);
    referee_power_ = DecodeOffsetPower(data.referee_power);
    superpower_output_max_ = DecodeDirectPower(data.superpower_output_max);
    output_capability_ = data.output_capability;
  }

  /**
   * @brief 获取底盘实际功率。
   *        Get the actual chassis power.
   *
   * @return 解码后的底盘实际功率，单位 W，离线时为 0。
   *         Decoded actual chassis power in W, 0 when offline.
   */
  float GetChassisPower()
  {
    if (!RefreshOnlineState())
    {
      return 0.0f;
    }

    return chassis_power_;
  }

  /**
   * @brief 获取超电输出能力比例。
   *        Get the output capability ratio of the capacitor.
   *
   * @return `output_capability / 255.0f`，离线时为 0。
   *         `output_capability / 255.0f`, 0 when offline.
   */
  float GetCapEnergy()
  {
    RefreshOnlineState();
    return static_cast<float>(output_capability_) / 255.0f;
  }

  /**
   * @brief 获取裁判系统总输出功率。
   *        Get the total referee output power.
   *
   * @return 解码后的裁判系统总输出功率，单位 W，离线时为 0。
   *         Decoded total referee output power in W, 0 when offline.
   */
  float GetRefereePower()
  {
    if (!RefreshOnlineState())
    {
      return 0.0f;
    }

    return referee_power_;
  }

  /**
   * @brief 获取超电可向 A 侧输出的最大功率。
   *        Get the maximum power the capacitor can deliver to side A.
   *
   * @return 最大输出功率，单位 W，离线时为 0。
   *         Maximum output power in W, 0 when offline.
   */
  float GetSuperPowerOutputMax()
  {
    if (!RefreshOnlineState())
    {
      return 0.0f;
    }

    return superpower_output_max_;
  }

  /**
   * @brief 获取超电侧当前功率限制。
   *        Get the power limit currently applied on the capacitor side.
   *
   * @return 功率限制原始值，离线时为 0。
   *         Raw power limit, 0 when offline.
   */
  uint8_t GetPowerLimit()
  {
    RefreshOnlineState();
    return power_limit_;
  }

  /**
   * @brief 获取超电在线状态。
   *        Get the online state of the capacitor.
   *
   * @return 在线为 true；离线或尚未收到状态帧为 false。
   *         True when online; false when offline or no status frame has been received
   *         yet.
   */
  bool IsOnline() { return RefreshOnlineState(); }

 private:
  static constexpr uint8_t ENABLE_CONV_MASK = 0x01;  ///< flags 中的 enableCONV 位
  ///< enableCONV bit of flags
  static constexpr uint32_t COMMAND_PERIOD_MS = 5;  ///< 控制帧最小发送间隔，单位 ms
  ///< Minimum control frame interval in ms
  static constexpr uint16_t SAME_FRAME_OFFLINE_COUNT = 200;  ///< 判为离线的连续相同帧数
  ///< Consecutive identical frames that mark the Module offline
  static constexpr float POWER_ENCODE_OFFSET = 16384.0f;  ///< 功率编码的零点偏移
  ///< Zero offset of the power encoding
  static constexpr float POWER_ENCODE_SCALE = 64.0f;  ///< 功率编码的缩放倍数
  ///< Scale of the power encoding

  /**
   * @brief 解码带零点偏移的功率字段。
   *        Decode a power field with a zero offset.
   *
   * @param encoded 协议中的功率编码值。
   *                Encoded power value of the protocol.
   * @return 功率，单位 W。
   *         Power in W.
   */
  static float DecodeOffsetPower(uint16_t encoded)
  {
    return (static_cast<float>(encoded) - POWER_ENCODE_OFFSET) / POWER_ENCODE_SCALE;
  }

  /**
   * @brief 解码直接给出功率值的字段。
   *        Decode a field that holds the power value directly.
   *
   * @param power 协议中的功率值。
   *              Power value of the protocol.
   * @return 功率，单位 W。
   *         Power in W.
   */
  static float DecodeDirectPower(uint16_t power) { return static_cast<float>(power); }

  /**
   * @brief 更新连续相同状态帧计数。
   *        Update the consecutive identical status frame count.
   *
   * @param data 当前状态帧。
   *             Current status frame.
   */
  void UpdateSameFrameCount(const StatusData& data)
  {
    if (!status_received_ ||
        std::memcmp(&data, &last_status_data_, sizeof(StatusData)) != 0)
    {
      last_status_data_ = data;
      same_frame_count_ = 1;
      return;
    }

    if (same_frame_count_ < SAME_FRAME_OFFLINE_COUNT)
    {
      ++same_frame_count_;
    }
  }

  /**
   * @brief CAN 接收回调，完成状态帧处理与控制帧发送。
   *        CAN receive callback that handles the status frame and sends the control
   *        frame.
   *
   * @param in_isr 是否在中断上下文中调用。
   *               Whether called from interrupt context.
   * @param self SuperPower 实例。
   *             SuperPower instance.
   * @param pack 接收到的 CAN 帧。
   *             Received CAN frame.
   */
  static void RxCallback(bool in_isr, SuperPower* self,
                         const LibXR::CAN::ClassicPack& pack)
  {
    UNUSED(in_isr);
    self->OnFeedbackFrame(pack);
  }

  /**
   * @brief 清空由状态帧更新的对外状态，保留缓存的裁判系统功率上限。
   *        Clear the exposed state updated by status frames, keeping the cached referee
   *        power limit.
   */
  void ClearStatus()
  {
    power_limit_ = 0;
    chassis_power_ = 0.0f;
    referee_power_ = 0.0f;
    superpower_output_max_ = 0.0f;
    output_capability_ = 0;
  }

  /**
   * @brief 按连续相同状态帧数量刷新在线状态，离线时清空对外状态。
   *        Refresh the online state from the consecutive identical frame count and clear
   *        the exposed state when offline.
   *
   * @return 在线为 true；尚未收到状态帧或连续相同帧达到阈值为 false。
   *         True when online; false when no status frame was received yet or the
   *         identical frame count reached the threshold.
   */
  bool RefreshOnlineState()
  {
    if (!status_received_)
    {
      ClearStatus();
      return false;
    }

    if (same_frame_count_ >= SAME_FRAME_OFFLINE_COUNT)
    {
      ClearStatus();
      return false;
    }

    return true;
  }

  /**
   * @brief 距上次发送至少 5 ms 时发送控制帧。
   *        Send the control frame when at least 5 ms have passed since the last one.
   *
   * @param now_ms 当前毫秒时间戳。
   *               Current timestamp in ms.
   */
  void SendCommandFrame(uint32_t now_ms)
  {
    const uint32_t LAST_TX_MS = last_command_tx_time_ms_;
    const auto NOW_TIMESTAMP = LibXR::MillisecondTimestamp(now_ms);
    const auto LAST_TX_TIMESTAMP = LibXR::MillisecondTimestamp(LAST_TX_MS);

    if ((NOW_TIMESTAMP - LAST_TX_TIMESTAMP).ToMillisecond() < COMMAND_PERIOD_MS)
    {
      return;
    }

    SendCommandFrame();
    last_command_tx_time_ms_ = now_ms;
  }

  /**
   * @brief 发送超电控制帧，打开 enableCONV 并写入最新的裁判系统底盘功率上限。
   *        Send the capacitor control frame with enableCONV set and the latest referee
   *        chassis power limit.
   */
  void SendCommandFrame()
  {
    CommandData command_data{};
    command_data.flags = ENABLE_CONV_MASK;
    command_data.referee_power_limit = referee_power_limit_;

    LibXR::CAN::ClassicPack tx_pack{};
    tx_pack.id = COMMAND_ID;
    tx_pack.type = LibXR::CAN::Type::STANDARD;
    tx_pack.dlc = sizeof(CommandData);
    static_assert(sizeof(CommandData) == 8, "CommandData must be 8 bytes for CAN");
    LibXR::Memory::FastCopy(tx_pack.data, &command_data, sizeof(CommandData));
    can_->AddMessage(tx_pack);
  }

  LibXR::CAN* can_;  ///< CAN 总线
  ///< CAN bus

  uint16_t referee_power_limit_ = 0;  ///< 缓存的裁判系统功率上限
  ///< Cached referee power limit
  StatusData last_status_data_{};  ///< 上一帧状态数据
  ///< Previous status data
  float chassis_power_ = 0.0f;  ///< 底盘实际功率，单位 W
  ///< Actual chassis power in W
  float referee_power_ = 0.0f;  ///< 裁判系统总功率，单位 W
  ///< Total referee power in W
  float superpower_output_max_ = 0.0f;  ///< 最大输出功率，单位 W
  ///< Maximum output power in W
  uint32_t last_command_tx_time_ms_ = 0;  ///< 上次发送控制帧的时间，单位 ms
  ///< Time of the last control frame in ms
  uint16_t same_frame_count_ = 0;  ///< 连续相同状态帧数量
  ///< Consecutive identical status frames
  uint8_t power_limit_ = 0;  ///< 功率限制原始值
  ///< Raw power limit
  uint8_t output_capability_ = 0;  ///< 输出能力原始值
  ///< Raw output capability
  bool status_received_ = false;  ///< 是否收到过状态帧
  ///< Whether a status frame was received
};
