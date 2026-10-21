// Unit tests for the ported TriggerSemanticAdapter
// (VaderHaptics tools/m3/haptics.cpp -> asb::haptics::TriggerSemanticAdapter).
#ifdef NDEBUG
#undef NDEBUG
#endif

#include "haptics/TriggerSemanticAdapter.h"

#include <cassert>
#include <cstdint>

namespace {

using asb::haptics::TriggerSemanticAdapter;

asb::dualsense::DualSenseFeedback makeFeedback() {
    asb::dualsense::DualSenseFeedback feedback{};
    feedback.kind = asb::dualsense::FeedbackKind::HidOutput;
    return feedback;
}

void setEffect(asb::dualsense::DualSenseFeedback& feedback, bool left,
               std::uint8_t mode, std::uint8_t startResistance = 0,
               std::uint8_t effectForce = 0) {
    auto& block = left ? feedback.leftTriggerEffect : feedback.rightTriggerEffect;
    block.fill(0);
    block[0] = mode;
    block[2] = startResistance;
    block[3] = effectForce;
}

} // namespace

int main() {
    // --- mode classification matches VaderHaptics ClassifyTriggerEffect ---
    assert(TriggerSemanticAdapter::classify(0) ==
           TriggerSemanticAdapter::EffectClass::Off);
    assert(TriggerSemanticAdapter::classify(5) ==
           TriggerSemanticAdapter::EffectClass::Off);
    assert(TriggerSemanticAdapter::classify(1) ==
           TriggerSemanticAdapter::EffectClass::Continuous);
    assert(TriggerSemanticAdapter::classify(2) ==
           TriggerSemanticAdapter::EffectClass::Breakpoint);
    assert(TriggerSemanticAdapter::classify(6) ==
           TriggerSemanticAdapter::EffectClass::Oscillation);
    assert(TriggerSemanticAdapter::classify(37) ==
           TriggerSemanticAdapter::EffectClass::Oscillation);
    assert(TriggerSemanticAdapter::classify(38) ==
           TriggerSemanticAdapter::EffectClass::Oscillation);
    assert(TriggerSemanticAdapter::classify(33) ==
           TriggerSemanticAdapter::EffectClass::Weapon);
    // 37/38 decimal == 0x25/0x26; both are oscillation, as the real captures show.
    assert(TriggerSemanticAdapter::classify(0x25) ==
           TriggerSemanticAdapter::EffectClass::Oscillation);
    assert(TriggerSemanticAdapter::classify(0x26) ==
           TriggerSemanticAdapter::EffectClass::Oscillation);
    // Values outside the known DualSense mode groups stay unsupported.
    assert(TriggerSemanticAdapter::classify(0x21) ==
           TriggerSemanticAdapter::EffectClass::Weapon);  // 33 == 0x21
    assert(TriggerSemanticAdapter::classify(0x11) ==
           TriggerSemanticAdapter::EffectClass::Unsupported);
    assert(TriggerSemanticAdapter::classify(0x12) ==
           TriggerSemanticAdapter::EffectClass::Unsupported);

    // --- 11-byte block decode ---
    std::array<std::uint8_t, 11> block{};
    block[0] = 0x02;
    block[2] = 0x19;
    block[3] = 0x5A;
    block[4] = 0x3C;
    block[5] = 0x01;
    block[6] = 0x02;
    block[7] = 0x03;
    block[9] = 0x23;
    const auto effect = TriggerSemanticAdapter::decodeEffect(block);
    assert(effect.mode == 0x02);
    assert(effect.startResistance == 0x19);
    assert(effect.effectForce == 0x5A);
    assert(effect.rangeForce == 0x3C);
    assert(effect.nearReleaseStrength == 0x01);
    assert(effect.nearMiddleStrength == 0x02);
    assert(effect.pressedStrength == 0x03);
    assert(effect.frequency == 0x23);

    // --- an idle adapter with no effect and no pull produces nothing ---
    TriggerSemanticAdapter adapter;
    auto silent = makeFeedback();
    auto drive = adapter.process(silent, 0, 0, 1000);
    assert(drive.leftTrigger == 0.0f);
    assert(drive.rightTrigger == 0.0f);

    // --- Continuous (bow draw) ramps with pull and is gated by the wall ---
    adapter.reset();
    auto continuous = makeFeedback();
    setEffect(continuous, false, 0x01, /*startResistance=*/0x40,
              /*effectForce=*/200);
    // Below the wall: nothing.
    drive = adapter.process(continuous, 0, 0x20, 10000);
    assert(drive.rightTrigger == 0.0f);
    // Past the wall and near full pull: visible drive, capped by limiterCap.
    drive = adapter.process(continuous, 0, 0xFF, 20000);
    assert(drive.rightTrigger > 0.10f);
    assert(drive.rightTrigger <= 0.50f);
    // Left side has no effect set.
    assert(drive.leftTrigger == 0.0f);

    // --- Breakpoint produces a finite pulse that decays ---
    adapter.reset();
    auto breakpoint = makeFeedback();
    setEffect(breakpoint, true, 0x02, 0x30, 200);
    // Cross the wall: a pulse starts.
    auto pulse = adapter.process(breakpoint, 0x40, 0, 100000);
    assert(pulse.leftTrigger > 0.0f);
    // Long after the pulse window, holding still past the wall gives nothing.
    auto decayed = adapter.process(breakpoint, 0x40, 0, 300000);
    assert(decayed.leftTrigger == 0.0f);

    // --- Unsupported modes stay silent but reset fallback state ---
    adapter.reset();
    auto unsupported = makeFeedback();
    setEffect(unsupported, true, 0x11);  // outside every known mode group
    // Drive the PCM path first so a fallback pulse would exist.
    for (auto& sample : unsupported.leftHapticSamples) sample = 32000;
    adapter.process(unsupported, 0xFF, 0, 1000);
    for (auto& sample : unsupported.leftHapticSamples) sample = 0;
    const auto quiet = adapter.process(unsupported, 0xFF, 0, 20000);
    assert(quiet.leftTrigger == 0.0f);

    // --- PCM-onset fallback: a gunshot with no trigger metadata still ticks ---
    adapter.reset();
    auto onset = makeFeedback();
    setEffect(onset, false, 0x00);  // Off -> fallback path
    auto quietBlock = adapter.process(onset, 0xFF, 0xFF, 1000);
    assert(quietBlock.rightTrigger == 0.0f);
    // Sudden loud block: the fallback should fire a trigger impulse.
    for (auto& sample : onset.rightHapticSamples) sample = 32767;
    const auto loud = adapter.process(onset, 0xFF, 0xFF, 2000);
    assert(loud.rightTrigger > 0.0f);
    assert(loud.rightTrigger <= 0.50f);

    // --- reset() clears lingering state ---
    adapter.reset();
    const auto afterReset = adapter.process(silent, 0, 0, 500000);
    assert(afterReset.leftTrigger == 0.0f);
    assert(afterReset.rightTrigger == 0.0f);

    return 0;
}
