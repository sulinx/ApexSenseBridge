#pragma once

#include "dualsense/DualSenseFeedback.h"

#include <cstdint>

namespace asb::haptics {

struct MotorLevels {
    std::uint8_t lowFrequency = 0;
    std::uint8_t highFrequency = 0;

    bool operator==(const MotorLevels&) const = default;
};

struct HapticConfig {
    // LOCAL PATCH: the original defaults were tuned against a loud reference
    // capture (left/right peak 65535, i.e. full scale). Measured on the same
    // Vader 5 Pro + Death Stranding 2 pair, a normal play session produced
    // peaks of only ~21 500 and energies of ~16 900, one third of that
    // reference. With a hard 12 % activation gate plus a 0.72 response curve
    // the whole session collapsed to a motor level of 60/255 and quiet
    // passages produced exactly 0, which reads to the player as "vibration is
    // gone". The gate is the real problem: it is a fixed 12 % cost that grows
    // relatively larger the quieter the signal is.
    //
    // Removing the gate and flattening the curve lifts that same session to
    // ~142/255 while the loud reference still saturates (238/255), so dynamic
    // range is preserved instead of being compressed away. Floors are dropped
    // to zero for the same reason: they were subtracted before normalisation
    // and cost another fixed slice of a quiet signal.
    std::uint16_t energyFloor = 0;
    std::uint16_t peakFloor = 0;
    std::uint16_t transientFloor = 0;
    double energyWeight = 0.16;
    double peakWeight = 0.50;
    double transientWeight = 0.34;
    double activationThreshold = 0.0;
    double responseCurve = 0.45;
    double outputGain = 1.0;
};

// VIIPER has already reduced each 240-sample (5 ms at 48 kHz) audio window to
// energy, peak, and rising-transient measurements. This processor performs the
// remaining perceptual mapping without allocations or additional buffering.
class HapticProcessor {
public:
    explicit HapticProcessor(HapticConfig config = {});

    [[nodiscard]] MotorLevels process(
        const dualsense::DualSenseFeedback& feedback) const noexcept;

private:
    HapticConfig config_;
};

} // namespace asb::haptics
