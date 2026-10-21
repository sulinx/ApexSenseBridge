#pragma once

#include "core/TriggerEffect.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace asb::flydigi {

// Vendor HID command interface documented/measured by OpenFlydigi.
constexpr std::uint16_t kVendorId = 0x37D7;
constexpr std::uint16_t kVendorUsagePage = 0xFFA0;
constexpr std::uint8_t kControllerProductFamily = 0x2; // PID >> 12

constexpr std::uint8_t kReportIdOut = 0x03;
constexpr std::uint8_t kReportIdIn = 0x04;

// Vader 5 Pro (f5, DeviceType 130) differences from the Apex 5 family, both
// measured on hardware by VaderHaptics and confirmed here:
//  * output reports carry report ID 0x00, not 0x03 (33-byte interface);
//  * the 0x12 rumble payload is 4 lanes in one frame
//    (left grip, right grip, left trigger, right trigger) instead of a single
//    low/high motor pair.
constexpr std::uint8_t kVader5ProDeviceType = 130;
constexpr std::uint8_t kReportIdOutVader = 0x00;
constexpr std::uint8_t kVaderRumblePayloadBytes = 4;
constexpr std::uint8_t kMagic0 = 0x5A;
constexpr std::uint8_t kMagic1 = 0xA5;
constexpr std::uint8_t kCmdGetInfo = 0x01;
// LOCAL PATCH: SDL's Flydigi V2 driver takes ownership with 0x1C "SDL" and
// renews it every 30 s. The Vader 5 Pro only starts its 0xEF operator stream
// after this ACQUIRE succeeds.
constexpr std::uint8_t kCmdAcquire = 0x1C;
constexpr std::uint8_t kCmdReadInputTransport = 0x10;
constexpr std::uint8_t kCmdSetInputTransport = 0x11;
constexpr std::uint8_t kCmdSetRumble = 0x12;
constexpr std::uint8_t kCmdOperatorData = 0xEF;
constexpr std::uint8_t kCmdSetForceTrigger = 81;
constexpr std::uint8_t kCmdProfileStatus = 0xA1;
constexpr std::uint8_t kCmdApplyProfile = 0xA2;
constexpr std::uint8_t kCmdReadRgbConfig = 0xA7;
constexpr std::uint8_t kCmdWriteRgbStart = 0xA8;
constexpr std::uint8_t kCmdWriteRgbPack = 0xA9;
constexpr std::uint8_t kRgbPacketSize = 20;
constexpr std::uint8_t kRgbPacketCount = 19;
constexpr std::size_t kRgbConfigSize = 380;
constexpr std::size_t kApex5LedCount = 12;
constexpr std::uint8_t kProfileSlotCount = 4;
constexpr std::size_t kReportSize = 32;

using Report = std::array<std::uint8_t, kReportSize>;

struct ProfileStatus {
    std::uint8_t rawSlot = 0;
    std::uint8_t slot = 0;
    bool switchBank = false;
};

struct TransportFlag {
    std::uint8_t raw = 0;

    [[nodiscard]] constexpr std::optional<bool> value() const noexcept {
        if (raw > 1) return std::nullopt;
        return raw == 1;
    }

    bool operator==(const TransportFlag&) const = default;
};

struct InputTransportStatus {
    bool controllerData = true;
    bool rawData = false;
    TransportFlag keyboardData{};
    TransportFlag mouseData{};
    TransportFlag thirdPartyControl{};
};

[[nodiscard]] Report buildForceTrigger(const TriggerEffect& effect, bool apply = true);
[[nodiscard]] Report buildForceTriggerRaw(const ForceTriggerCommand& command,
                                          bool apply = true);
[[nodiscard]] Report buildNormal(TriggerSide side);
[[nodiscard]] Report buildRumble(std::uint8_t lowFrequencyMotor,
                                 std::uint8_t highFrequencyMotor);
// Vader 5 Pro uses report ID 0x00 for every vendor command, including the
// read-only identity probe. Same 0x5A 0xA5 body as Apex5Identity::buildRequest.
[[nodiscard]] Report buildVaderIdentityRequest();
// Vader 5 Pro four-lane rumble: the same 0x12 command with report ID 0x00 and a
// four-byte payload. Apex-family frames leave lanes 7/8 zero, so the extra two
// bytes are inert on Apex hardware.
[[nodiscard]] Report buildVaderRumble(std::uint8_t leftGrip,
                                      std::uint8_t rightGrip,
                                      std::uint8_t leftTrigger,
                                      std::uint8_t rightTrigger);
[[nodiscard]] Report buildReadRgbConfig(std::uint8_t slot = 0, std::uint8_t packetSize = kRgbPacketSize);
[[nodiscard]] Report buildWriteRgbStart(std::uint8_t slot, std::uint8_t startIndex, std::uint8_t packetCount, std::uint8_t packetSize = kRgbPacketSize);
[[nodiscard]] Report buildWriteRgbPack(std::uint8_t packetIndex, std::span<const std::uint8_t> data);
[[nodiscard]] std::array<std::uint8_t, kRgbConfigSize> buildStaticRgbPayload(
    std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t brightness = 100);
[[nodiscard]] Report buildProfileStatusRequest();
[[nodiscard]] std::optional<Report> buildApplyProfile(std::uint8_t slot);
[[nodiscard]] Report buildInputTransportStatusRequest();
[[nodiscard]] Report buildSetInputTransport(bool controllerData, bool rawData);
// LOCAL PATCH: 0x1C ACQUIRE / release, matching SDL's Flydigi V2 driver. The
// Vader answers with checksum 0x00 (a literal, not a running sum), so it is
// built explicitly instead of via buildChecksummedCommand().
[[nodiscard]] Report buildAcquire(bool acquire);
[[nodiscard]] bool isProfileCommandReply(
    std::span<const std::uint8_t> report, std::uint8_t command) noexcept;
[[nodiscard]] std::optional<ProfileStatus> parseProfileStatus(
    std::span<const std::uint8_t> report) noexcept;
[[nodiscard]] std::optional<InputTransportStatus> parseInputTransportStatus(
    std::span<const std::uint8_t> report) noexcept;
[[nodiscard]] bool isControllerProduct(std::uint16_t productId) noexcept;

} // namespace asb::flydigi
