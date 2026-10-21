#pragma once

#include "dualsense/DualSenseFeedback.h"
#include "haptics/HapticProcessor.h"

#include <array>
#include <cstdint>

namespace asb::haptics {

// Ported from VaderHaptics tools/m3/haptics.cpp::TriggerSemanticAdapter
// (MIT, (c) 2026 VaderHaptics contributors).
//
// Purpose: a DualSense adaptive-trigger effect is *resistance* metadata. The
// Vader 5 Pro has no resistance actuator, only a trigger rumble motor, so the
// metadata is converted into bounded trigger vibration gated by the real
// trigger position. It never attempts to emulate resistance.
//
// Adaptation to ApexSenseBridge:
//  * VaderHaptics decodes 32 interleaved int8 stereo samples per 10.67 ms block;
//    ApexSenseBridge receives 8 decimated int16 stereo samples per block from
//    VIIPER, so the energy/crest estimators read those instead.
//  * Trigger metadata arrives pre-split as 11-byte DualSense effect blocks.
class TriggerSemanticAdapter {
public:
    struct Config {
        // Effect classes mirror the DualSense mode byte groups used by
        // VaderHaptics ClassifyTriggerEffect().
        double limiterCap = 0.50;        // never exceed this per-lane drive
        double impactThreshold = 0.018;  // reject tiny block-to-block onsets
        double impactFullScale = 0.22;   // onset that reaches full impact target
        double impactReleaseS = 0.050;   // finite impact tail, not a held buzz
        double triggerFallbackGain = 0.38;
        double triggerFallbackCap = 96.0 / 255.0;
        std::uint8_t triggerFallbackDeadzone = 32;
        double triggerFallbackOnsetGate = 0.12;
        double triggerFallbackPulseS = 0.055;
        double triggerFallbackRefractoryS = 0.085;
        double triggerFallbackGamma = 0.68;
    };

    // DualSense trigger mode byte -> coarse family (VaderHaptics
    // ClassifyTriggerEffect).
    enum class EffectClass {
        Off,
        Continuous,
        Breakpoint,
        Weapon,
        Oscillation,
        Unsupported,
    };

    static EffectClass classify(std::uint8_t mode) noexcept;

    explicit TriggerSemanticAdapter(Config config = {});

    void reset();

    // Decode one 11-byte DualSense effect block into the subset this adapter
    // needs. Returns false when the block is empty/all-zero.
    struct Effect {
        std::uint8_t mode = 0;
        std::uint8_t startResistance = 0;
        std::uint8_t effectForce = 0;
        std::uint8_t rangeForce = 0;
        std::uint8_t nearReleaseStrength = 0;
        std::uint8_t nearMiddleStrength = 0;
        std::uint8_t pressedStrength = 0;
        std::uint8_t frequency = 0;

        bool operator==(const Effect&) const = default;
    };
    static Effect decodeEffect(const std::array<std::uint8_t, 11>& block) noexcept;

    // Produce per-side trigger drive in [0, limiterCap].
    //  leftPosition / rightPosition are the raw 0..255 trigger axes.
    //  nowUs is a monotonic microsecond clock.
    struct Drive {
        float leftTrigger = 0.0f;
        float rightTrigger = 0.0f;
    };
    [[nodiscard]] Drive process(const dualsense::DualSenseFeedback& feedback,
                                std::uint8_t leftPosition,
                                std::uint8_t rightPosition,
                                std::int64_t nowUs);

private:
    struct SideState {
        Effect lastEffect;
        std::uint8_t lastPosition = 0;
        std::int64_t pulseUntilUs = 0;
        double previousSourceEnergy = 0.0;
        double fallbackPulse = 0.0;
        std::int64_t fallbackPulseStartUs = 0;
        std::int64_t fallbackPulseUntilUs = 0;
        std::int64_t fallbackRearmUs = 0;
    };

    struct Signal {
        double energy = 0.0;
        double crest = 0.0;
    };

    [[nodiscard]] Signal measure(const std::array<std::int16_t, 8>& samples) const noexcept;
    float processSide(const Effect& effect, std::uint8_t position,
                      std::int64_t nowUs, const Signal& signal, SideState& state);

    Config config_;
    double fallbackReleaseCoeff_ = 0.0;
    SideState left_;
    SideState right_;
};

} // namespace asb::haptics
