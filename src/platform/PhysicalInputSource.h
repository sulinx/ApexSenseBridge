#pragma once

#include "core/DeviceInfo.h"
#include "dualsense/DualSenseInput.h"
#include "platform/PhysicalInputActivity.h"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace asb::platform {

enum class PhysicalInputStatus {
    State,
    Timeout,
    Disconnected,
    Error,
};

struct PhysicalInputSourceStats {
    std::uint64_t reports = 0;
    std::uint64_t timeouts = 0;
    std::uint64_t parseFailures = 0;
    std::uint64_t mappedReports = 0;
    std::uint64_t vendorReports = 0;
    std::uint64_t vendorStates = 0;
    std::uint64_t vendorParseFailures = 0;
    std::uint64_t vendorReadFailures = 0;
    std::uint64_t triggerSourceSwitches = 0;
    std::uint64_t vendorMaximumGapMs = 0;
    std::uint64_t vendorStaleTriggerGaps = 0;
    PhysicalInputActivityStats activity{};
};

// Reads the complete physical APEX game state for the mandatory
// APEX -> bridge -> virtual DualSense path. A game must never consume this
// physical source directly while a DualSense bridge session is active.
class PhysicalInputSource {
public:
    virtual ~PhysicalInputSource() = default;

    virtual PhysicalInputStatus waitForState(
        dualsense::DualSenseInputState& state,
        std::chrono::milliseconds timeout,
        std::string& error) = 0;

    [[nodiscard]] virtual std::string_view backendName() const noexcept = 0;
    [[nodiscard]] virtual bool eventDriven() const noexcept = 0;
    [[nodiscard]] virtual PhysicalInputSourceStats stats() const noexcept = 0;
    // A centered HID trigger axis cannot represent LT+RT together. Such a
    // source is only gameplay-ready while its independent stream is live.
    [[nodiscard]] virtual bool requiresIndependentTriggers() const noexcept { return false; }
    [[nodiscard]] virtual bool independentTriggersReady() const noexcept { return true; }
    // LOCAL PATCH: re-arm the underlying vendor reads after a silence that the
    // freshness watchdog would otherwise treat as a dead stream. Measured on a
    // Vader 5 Pro over the 2.4 GHz dongle: operator frames stall for 1.6-3.2 s
    // while the controller is being used heavily, then resume on their own. The
    // stall is a device/dongle hiccup, not a disconnect, so aborting the session
    // is wrong; re-arming cancels nothing when the read is genuinely in flight
    // and resurrects it when a completion was consumed elsewhere.
    // Returns true when at least one stream was re-armed.
    virtual bool recoverStream() noexcept { return false; }
    virtual void setBatteryState(std::uint8_t batteryPercent, std::uint8_t chargeState) noexcept {
        (void)batteryPercent;
        (void)chargeState;
    }
};

// Uses the game-controller HID collection belonging to the same Windows
// container as apexVendorInterface. --xinput-index remains an advanced escape
// hatch and explicitly selects the polling fallback.
// vaderOperatorReport: when true the vendor stream is decoded with the Vader 5
// Pro 0xEF layout instead of the Apex 5 layout (same interface, different frame).
// Callers pass the already-verified identity.
std::unique_ptr<PhysicalInputSource> openPhysicalInputSource(
    const HidDeviceInfo& apexVendorInterface,
    std::optional<unsigned int> requestedXInputIndex,
    std::string& error,
    bool vaderOperatorReport = false);

} // namespace asb::platform
