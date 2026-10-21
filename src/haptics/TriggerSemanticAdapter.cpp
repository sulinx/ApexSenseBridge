#include "haptics/TriggerSemanticAdapter.h"

#include <algorithm>
#include <cmath>

namespace asb::haptics {
namespace {

// ApexSenseBridge delivers 8 decimated 1 ms stereo samples per feedback frame
// (DualSenseFeedback::leftHapticSamples / rightHapticSamples), so each block
// spans 8 ms. VaderHaptics uses 32 samples at 3 kHz (10.667 ms); the decay
// coefficients below are derived from this project's own block duration.
constexpr double kBlockSeconds = 8.0 / 1000.0;
constexpr double kSamplesPerBlock = 8.0;

} // namespace

TriggerSemanticAdapter::EffectClass TriggerSemanticAdapter::classify(
    std::uint8_t mode) noexcept {
    switch (mode) {
    case 0:
    case 5:
        return EffectClass::Off;
    case 1:
        return EffectClass::Continuous;
    case 2:
        return EffectClass::Breakpoint;
    case 6:
    case 37:
    case 38:
        return EffectClass::Oscillation;
    case 33:
        return EffectClass::Weapon;
    default:
        return EffectClass::Unsupported;
    }
}

TriggerSemanticAdapter::Effect TriggerSemanticAdapter::decodeEffect(
    const std::array<std::uint8_t, 11>& block) noexcept {
    // DualSense 11-byte adaptive-trigger block: byte 0 is the mode; the
    // remaining bytes carry per-mode parameters. The mapping below matches the
    // parameter order VaderHaptics reads for the same modes.
    Effect effect;
    effect.mode = block[0];
    effect.startResistance = block[2];
    effect.effectForce = block[3];
    effect.rangeForce = block[4];
    effect.nearReleaseStrength = block[5];
    effect.nearMiddleStrength = block[6];
    effect.pressedStrength = block[7];
    effect.frequency = block[9];
    return effect;
}

TriggerSemanticAdapter::TriggerSemanticAdapter(Config config) : config_(config) {
    fallbackReleaseCoeff_ =
        1.0 - std::exp(-kBlockSeconds / std::max(config_.impactReleaseS, 1e-6));
}

void TriggerSemanticAdapter::reset() {
    left_ = SideState{};
    right_ = SideState{};
}

TriggerSemanticAdapter::Signal TriggerSemanticAdapter::measure(
    const std::array<std::int16_t, 8>& samples) const noexcept {
    // VaderHaptics computes energy = 0.35*peak + 0.65*rms and
    // crest = clamp((peak - mean) / peak, 0, 1) over one block. The same shape
    // is kept here so the downstream thresholds stay meaningful.
    double peak = 0.0;
    double sumAbs = 0.0;
    double sumSquares = 0.0;
    for (const auto sample : samples) {
        const double normalized = static_cast<double>(sample) / 32767.0;
        peak = std::max(peak, std::abs(normalized));
        sumAbs += std::abs(normalized);
        sumSquares += normalized * normalized;
    }
    const double mean = sumAbs / kSamplesPerBlock;
    const double rms = std::sqrt(sumSquares / kSamplesPerBlock);
    Signal signal;
    signal.energy = 0.35 * peak + 0.65 * rms;
    signal.crest =
        peak > 0.0 ? std::clamp((peak - mean) / peak, 0.0, 1.0) : 0.0;
    return signal;
}

float TriggerSemanticAdapter::processSide(const Effect& effect,
                                          std::uint8_t position,
                                          std::int64_t nowUs,
                                          const Signal& signal,
                                          SideState& state) {
    const EffectClass family = classify(effect.mode);
    const bool changed = !(effect == state.lastEffect);
    const auto threshold = static_cast<std::uint8_t>(
        std::clamp<int>(effect.startResistance, 8, 224));
    const bool crossed = state.lastPosition < threshold && position >= threshold;
    const double pull = static_cast<double>(position) / 255.0;
    const double strength =
        static_cast<double>(std::max({effect.effectForce, effect.rangeForce,
                                      effect.nearReleaseStrength,
                                      effect.nearMiddleStrength,
                                      effect.pressedStrength})) /
        255.0;

    float output = 0.0f;
    switch (family) {
    case EffectClass::Off: {
        // No trigger metadata: fall back to a PCM-onset impulse so a gunshot
        // still produces a trigger tick when the game drives haptics only.
        state.pulseUntilUs = 0;
        const double onset = std::max(0.0, signal.energy - state.previousSourceEnergy);
        state.previousSourceEnergy = signal.energy;
        const double linear =
            std::max(0.0, (onset - config_.impactThreshold) /
                              (config_.impactFullScale - config_.impactThreshold));
        double target = linear / (1.0 + linear);
        target *= 0.55 + 0.45 * signal.crest;
        bool startedFallbackPulse = false;
        if (target >= config_.triggerFallbackOnsetGate &&
            nowUs >= state.fallbackRearmUs) {
            state.fallbackPulse = target;
            startedFallbackPulse = true;
            state.fallbackPulseStartUs = nowUs;
            state.fallbackPulseUntilUs =
                nowUs + static_cast<std::int64_t>(config_.triggerFallbackPulseS * 1e6);
            state.fallbackRearmUs =
                nowUs + static_cast<std::int64_t>(config_.triggerFallbackRefractoryS * 1e6);
        }
        if (!startedFallbackPulse && nowUs < state.fallbackPulseUntilUs) {
            state.fallbackPulse +=
                (0.0 - state.fallbackPulse) * fallbackReleaseCoeff_;
        }
        else if (nowUs >= state.fallbackPulseUntilUs) {
            state.fallbackPulse = 0.0;
            state.fallbackPulseStartUs = 0;
        }
        if (position >= config_.triggerFallbackDeadzone &&
            nowUs < state.fallbackPulseUntilUs && state.fallbackPulse > 0.0) {
            const double gate =
                static_cast<double>(position - config_.triggerFallbackDeadzone) /
                std::max(1.0,
                         255.0 - static_cast<double>(config_.triggerFallbackDeadzone));
            const double durationUs =
                std::max(1.0, config_.triggerFallbackPulseS * 1e6);
            const double phase = std::clamp(
                static_cast<double>(nowUs - state.fallbackPulseStartUs) / durationUs,
                0.0, 1.0);
            const double frontLoaded = std::exp(-3.0 * phase);
            const double shaped =
                std::pow(state.fallbackPulse, config_.triggerFallbackGamma);
            output = static_cast<float>(
                std::min(config_.triggerFallbackCap,
                         config_.triggerFallbackGain * shaped * frontLoaded *
                             std::sqrt(gate)));
        }
        break;
    }
    case EffectClass::Unsupported:
        state.pulseUntilUs = 0;
        state.previousSourceEnergy = signal.energy;
        state.fallbackPulse = 0.0;
        state.fallbackPulseStartUs = 0;
        state.fallbackPulseUntilUs = 0;
        state.fallbackRearmUs = 0;
        break;
    case EffectClass::Continuous:
        // Sustained resistance (e.g. bow draw): rumble proportionally to pull.
        if (position >= threshold) {
            output = static_cast<float>((0.12 + 0.20 * strength) * pull);
        }
        break;
    case EffectClass::Breakpoint:
    case EffectClass::Weapon:
        // Wall / click: one short kick when the pull crosses the wall.
        if (crossed || (changed && position >= threshold)) {
            state.pulseUntilUs =
                nowUs + (family == EffectClass::Weapon ? 65000 : 45000);
        }
        if (position >= threshold && nowUs < state.pulseUntilUs) {
            output = static_cast<float>(0.30 + 0.18 * strength);
        }
        break;
    case EffectClass::Oscillation: {
        // Machine-gun / galloping: square-ish buzz whose rate follows the
        // DualSense frequency parameter.
        if (position >= threshold) {
            const double frequencyHz =
                8.0 + 22.0 * static_cast<double>(effect.frequency) / 255.0;
            const auto periodUs = static_cast<std::int64_t>(1e6 / frequencyHz);
            const bool on = periodUs > 0 && (nowUs % periodUs) < periodUs / 2;
            if (on) output = static_cast<float>(0.18 + 0.27 * strength);
        }
        break;
    }
    }

    state.lastEffect = effect;
    state.lastPosition = position;
    return std::min(output, static_cast<float>(config_.limiterCap));
}

TriggerSemanticAdapter::Drive TriggerSemanticAdapter::process(
    const dualsense::DualSenseFeedback& feedback, std::uint8_t leftPosition,
    std::uint8_t rightPosition, std::int64_t nowUs) {
    const auto leftEffect = decodeEffect(feedback.leftTriggerEffect);
    const auto rightEffect = decodeEffect(feedback.rightTriggerEffect);
    const auto leftSignal = measure(feedback.leftHapticSamples);
    const auto rightSignal = measure(feedback.rightHapticSamples);

    Drive drive;
    drive.leftTrigger = processSide(leftEffect, leftPosition, nowUs, leftSignal, left_);
    drive.rightTrigger =
        processSide(rightEffect, rightPosition, nowUs, rightSignal, right_);
    return drive;
}

} // namespace asb::haptics
