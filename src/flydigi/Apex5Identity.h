#pragma once

#include "flydigi/Apex5Protocol.h"

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace asb::flydigi {

enum class ApexProtocol {
    LegacyV1,
    CurrentV2,
    RealtimeV3,
};

class Apex5Identity {
public:
    [[nodiscard]] static Report buildRequest();
    [[nodiscard]] static std::optional<Apex5Identity> parseReply(
        std::span<const std::uint8_t> report);
    // Vader 5 Pro GET_INFO reply: bare 5A A5 01 frame (optionally with a stray
    // leading 0x00) carrying DeviceType at [offset+3].
    [[nodiscard]] static std::optional<Apex5Identity> parseVaderReply(
        std::span<const std::uint8_t> report);
    [[nodiscard]] static std::optional<Apex5Identity> parseApex4Reply(
        std::span<const std::uint8_t> report);
    [[nodiscard]] static std::optional<Apex5Identity> parseApex6Reply(
        std::span<const std::uint8_t> report);
    [[nodiscard]] static bool isApex4DeviceType(std::uint8_t deviceType) noexcept;
    [[nodiscard]] static bool isApex5DeviceType(std::uint8_t deviceType) noexcept;
    [[nodiscard]] static bool isVader5ProDeviceType(std::uint8_t deviceType) noexcept;

    [[nodiscard]] ApexProtocol protocol() const noexcept { return protocol_; }
    [[nodiscard]] std::uint8_t deviceType() const noexcept { return deviceType_; }
    [[nodiscard]] std::uint8_t connectionTypeRaw() const noexcept { return connectionType_; }
    [[nodiscard]] bool isWired() const noexcept {
        return protocol_ == ApexProtocol::RealtimeV3
            ? connectionType_ == 0
            : connectionType_ == 1;
    }
    [[nodiscard]] std::string_view connectionName() const noexcept;
    [[nodiscard]] bool isApex4() const noexcept {
        return protocol_ == ApexProtocol::LegacyV1 && isApex4DeviceType(deviceType_);
    }
    [[nodiscard]] bool isApex5() const noexcept {
        return protocol_ == ApexProtocol::CurrentV2 && isApex5DeviceType(deviceType_);
    }
    // Vader 5 Pro (f5, DeviceType 130) speaks the same 0x5A 0xA5 command frame as
    // the Apex5 V2 family but has no FORCEADAPT trigger hardware, so it must
    // never reach the adaptive-trigger path. See kVader5ProDeviceType.
    [[nodiscard]] bool isVader5Pro() const noexcept {
        return protocol_ == ApexProtocol::CurrentV2 && deviceType_ == kVader5ProDeviceType;
    }
    [[nodiscard]] bool isApex6() const noexcept {
        return protocol_ == ApexProtocol::RealtimeV3;
    }
    // Vader 5 Pro carries two grip plus two trigger rumble lanes and no trigger
    // resistance, so it is trigger-capable for input only.
    [[nodiscard]] bool supportsAdaptiveTriggers() const noexcept {
        return isApex4() || isApex5();
    }
    [[nodiscard]] bool supportsFourLaneVibration() const noexcept {
        return isVader5Pro();
    }
    [[nodiscard]] bool supportsRealtimeHaptics() const noexcept {
        return isApex6() && (features_ & 0x80U) != 0 &&
               (features_ & 0x10U) != 0;
    }
    [[nodiscard]] std::uint8_t features() const noexcept { return features_; }
    [[nodiscard]] bool hasBatteryLevel() const noexcept {
        return protocol_ == ApexProtocol::CurrentV2;
    }
    [[nodiscard]] std::uint8_t batteryLevel() const noexcept { return batteryLevel_; }
    [[nodiscard]] bool isCharging() const noexcept { return charging_; }
    [[nodiscard]] std::uint8_t batteryPercent() const noexcept;
    [[nodiscard]] std::uint8_t chargeState() const noexcept;
    [[nodiscard]] std::uint16_t firmwareVersion() const noexcept { return firmwareVersion_; }
    [[nodiscard]] std::string describe() const;

private:
    Apex5Identity(ApexProtocol protocol,
                  std::uint8_t deviceType,
                  std::uint8_t connectionType,
                  std::uint8_t batteryLevel,
                  bool charging,
                  std::uint16_t firmwareVersion,
                  std::uint8_t features = 0) noexcept;

    ApexProtocol protocol_ = ApexProtocol::CurrentV2;
    std::uint8_t deviceType_ = 0;
    std::uint8_t connectionType_ = 0;
    std::uint8_t batteryLevel_ = 0;
    bool charging_ = false;
    std::uint16_t firmwareVersion_ = 0;
    std::uint8_t features_ = 0;
};

} // namespace asb::flydigi
