#include "flydigi/Apex5Input.h"

#include "flydigi/Apex5Protocol.h"
#include "platform/XInputMapping.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <limits>

namespace asb::flydigi {
namespace {

std::int16_t signedLittleEndian(std::span<const std::uint8_t> report,
                                std::size_t offset) noexcept {
    const auto raw = static_cast<std::uint16_t>(
        report[offset] |
        static_cast<std::uint16_t>(report[offset + 1]) << 8U);
    return static_cast<std::int16_t>(raw);
}

std::int16_t scaleAccelerometer(std::int16_t value) noexcept {
    // The APEX 5 reports approximately 4096 raw units per g. The virtual
    // DualSense calibration advertises 10000 raw units per g, so preserve
    // physical acceleration by applying the exact 10000/4096 ratio.
    constexpr std::int32_t kNumerator = 625;
    constexpr std::int32_t kDenominator = 256;
    const auto scaled = static_cast<std::int32_t>(value) * kNumerator /
                        kDenominator;
    return static_cast<std::int16_t>((std::clamp)(
        scaled,
        static_cast<std::int32_t>((std::numeric_limits<std::int16_t>::min)()),
        static_cast<std::int32_t>((std::numeric_limits<std::int16_t>::max)())));
}

} // namespace

std::optional<dualsense::DualSenseInputState>
decodeApex5InputReport(std::span<const std::uint8_t> report,
                       std::uint8_t batteryPercent,
                       std::uint8_t chargeState) noexcept {
    if (report.size() < 30 || report[0] != kReportIdIn ||
        report[1] != kMagic0 || report[2] != kMagic1 ||
        report[3] != kCmdOperatorData) {
        return std::nullopt;
    }

    platform::XInputSnapshot snapshot{};
    snapshot.leftX = signedLittleEndian(report, 4);
    snapshot.leftY = signedLittleEndian(report, 6);
    snapshot.rightX = signedLittleEndian(report, 8);
    snapshot.rightY = signedLittleEndian(report, 10);
    snapshot.leftTrigger = report[16];
    snapshot.rightTrigger = report[17];

    const auto primary = report[12];
    const auto secondary = report[13];
    if (primary & 0x01) snapshot.buttons |= platform::xinputButton::kDpadUp;
    if (primary & 0x02) snapshot.buttons |= platform::xinputButton::kDpadRight;
    if (primary & 0x04) snapshot.buttons |= platform::xinputButton::kDpadDown;
    if (primary & 0x08) snapshot.buttons |= platform::xinputButton::kDpadLeft;
    if (primary & 0x10) snapshot.buttons |= platform::xinputButton::kA;
    if (primary & 0x20) snapshot.buttons |= platform::xinputButton::kB;
    if (primary & 0x40) snapshot.buttons |= platform::xinputButton::kBack;
    if (primary & 0x80) snapshot.buttons |= platform::xinputButton::kX;
    if (secondary & 0x01) snapshot.buttons |= platform::xinputButton::kY;
    if (secondary & 0x02) snapshot.buttons |= platform::xinputButton::kStart;
    if (secondary & 0x04) snapshot.buttons |= platform::xinputButton::kLeftShoulder;
    if (secondary & 0x08) snapshot.buttons |= platform::xinputButton::kRightShoulder;
    if (secondary & 0x40) snapshot.buttons |= platform::xinputButton::kLeftThumb;
    if (secondary & 0x80) snapshot.buttons |= platform::xinputButton::kRightThumb;

    auto state = platform::mapXInputState(snapshot);
    if (report[15] & 0x08) state.buttons |= dualsense::button::kPs;
    state.gyroX = signedLittleEndian(report, 18);
    state.gyroY = signedLittleEndian(report, 20);
    state.gyroZ = signedLittleEndian(report, 22);
    state.accelX = scaleAccelerometer(signedLittleEndian(report, 24));
    state.accelY = scaleAccelerometer(signedLittleEndian(report, 26));
    state.accelZ = scaleAccelerometer(signedLittleEndian(report, 28));
    state.batteryPercent = batteryPercent;
    state.chargeState = chargeState;
    return state;
}

std::optional<dualsense::DualSenseInputState>
decodeVaderInputReport(std::span<const std::uint8_t> report,
                       std::uint8_t batteryPercent,
                       std::uint8_t chargeState) noexcept {
    // LOCAL PATCH: Vader 5 Pro operator report (command 0xEF).
    // VaderHaptics tolerates a stray leading byte, so normalize it first.
    std::size_t offset = 0;
    if (report.size() > 1 && report[0] != kMagic0 && report[1] == kMagic0) {
        offset = 1;
    }
    if (report.size() < offset + 18 ||
        report[offset] != kMagic0 || report[offset + 1] != kMagic1 ||
        report[offset + 2] != kCmdOperatorData) {
        return std::nullopt;
    }
    const auto at = [&](std::size_t index) -> std::uint8_t {
        return report[offset + index];
    };

    platform::XInputSnapshot snapshot{};
    snapshot.leftX = signedLittleEndian(report, offset + 3);
    // LOCAL PATCH: do NOT negate Y here.
    //
    // This negation was copied from VaderHaptics' DecodeVaderReport, which works
    // in its own screen-space convention. Downstream, mapXInputState() already
    // converts a signed XInput axis into the DualSense byte axis with
    // axisToByte(value, invert = true) for both ly and ry, so negating here as
    // well inverted the pitch twice and delivered a backward stick. It stayed
    // invisible for as long as the mapped HID collection was merged over every
    // vendor frame (mergeApex5MappedControls overwrote both sticks with the
    // collection's own values); once the vendor frame became the sole source the
    // left stick read forward/backward reversed in game.
    //
    // The Apex 5 branch above (lines ~57-60) never negated, and it feeds the same
    // mapper, which is the direct evidence that the extra negation is wrong: the
    // raw Vader report already uses the standard XInput sign for Y.
    snapshot.leftY = signedLittleEndian(report, offset + 5);
    snapshot.rightX = signedLittleEndian(report, offset + 7);
    snapshot.rightY = signedLittleEndian(report, offset + 9);
    snapshot.leftTrigger = at(15);
    snapshot.rightTrigger = at(16);

    // Face/vendor buttons are a 16-bit little-endian mask starting at [11].
    // Bit assignments match VaderHaptics NormalizeMi01State().
    const auto face = static_cast<std::uint16_t>(
        at(11) | (static_cast<std::uint16_t>(at(12)) << 8U));
    if (face & 0x0010) snapshot.buttons |= platform::xinputButton::kA;
    if (face & 0x0020) snapshot.buttons |= platform::xinputButton::kB;
    if (face & 0x0040) snapshot.buttons |= platform::xinputButton::kBack;
    if (face & 0x0080) snapshot.buttons |= platform::xinputButton::kX;
    if (face & 0x0100) snapshot.buttons |= platform::xinputButton::kY;
    if (face & 0x0200) snapshot.buttons |= platform::xinputButton::kStart;
    if (face & 0x0400) snapshot.buttons |= platform::xinputButton::kLeftShoulder;
    if (face & 0x0800) snapshot.buttons |= platform::xinputButton::kRightShoulder;
    if (face & 0x4000) snapshot.buttons |= platform::xinputButton::kLeftThumb;
    if (face & 0x8000) snapshot.buttons |= platform::xinputButton::kRightThumb;

    // Raw hat is a four-direction bitmask, not an angle (dpad_map.h).
    const auto hat = static_cast<std::uint8_t>(at(11) & 0x0F);
    if (hat & 0x01) snapshot.buttons |= platform::xinputButton::kDpadUp;
    if (hat & 0x04) snapshot.buttons |= platform::xinputButton::kDpadDown;
    if (hat & 0x08) snapshot.buttons |= platform::xinputButton::kDpadLeft;
    if (hat & 0x02) snapshot.buttons |= platform::xinputButton::kDpadRight;

    auto state = platform::mapXInputState(snapshot);
    state.batteryPercent = batteryPercent;
    state.chargeState = chargeState;
    // The Vader operator report carries no IMU; leave gyro/accel at zero rather
    // than inventing motion data.
    return state;
}

void mergeApex5MappedControls(
    dualsense::DualSenseInputState& vendorState,
    const dualsense::DualSenseInputState& mappedState) noexcept {
    vendorState.lx = mappedState.lx;
    vendorState.ly = mappedState.ly;
    vendorState.rx = mappedState.rx;
    vendorState.ry = mappedState.ry;
    vendorState.l2 = mappedState.l2;
    vendorState.r2 = mappedState.r2;
    vendorState.dpad = mappedState.dpad;
    const auto psButton = vendorState.buttons & dualsense::button::kPs;
    vendorState.buttons = static_cast<std::uint16_t>(
        mappedState.buttons | psButton);
}

dualsense::DualSenseInputState composeApex5InputState(
    const dualsense::DualSenseInputState& mappedState,
    const std::optional<dualsense::DualSenseInputState>& vendorState,
    std::uint8_t batteryPercent,
    std::uint8_t chargeState) noexcept {
    Apex5InputComposer composer;
    return composer.compose(mappedState, vendorState,
                            Apex5InputComposer::Clock::duration::zero(),
                            batteryPercent, chargeState);
}

dualsense::DualSenseInputState Apex5InputComposer::compose(
    const dualsense::DualSenseInputState& mappedState,
    const std::optional<dualsense::DualSenseInputState>& vendorState,
    Clock::duration vendorTriggerAge,
    std::uint8_t batteryPercent,
    std::uint8_t chargeState) noexcept {
    auto state = vendorState.value_or(mappedState);
    mergeApex5MappedControls(state, mappedState);
    bool independent = false;
    if (vendorState) {
        const bool fresh = vendorTriggerAge <= kTriggerFreshness;
        if (independent_) {
            const int mappedBalance = static_cast<int>(mappedState.l2) - mappedState.r2;
            const int vendorBalance = static_cast<int>(vendorState->l2) - vendorState->r2;
            const bool confirmed = fresh ||
                std::abs(mappedBalance - vendorBalance) <= kStaleTriggerAgreement;
            independent = confirmed &&
                vendorState->l2 >= kIndependentReleaseFloor &&
                vendorState->r2 >= kIndependentReleaseFloor;
        } else {
            independent = fresh &&
                vendorState->l2 > kIndependentEngageFloor &&
                vendorState->r2 > kIndependentEngageFloor;
        }
    }
    if (independent != independent_) {
        independent_ = independent;
        ++switches_;
    }
    if (independent_) {
        platform::mergeIndependentTriggers(
            vendorState->l2, vendorState->r2, state);
    }
    state.batteryPercent = batteryPercent;
    state.chargeState = chargeState;
    return state;
}

} // namespace asb::flydigi
