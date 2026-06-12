#pragma once
// BigKnobDsp.hpp — BigKnob processing core, host-agnostic (no DPF).
// Altec 9069B model: SVF TPT (2p) -> one-pole TPT (1p) = 18 dB/oct,
// HP/LP crossfaded outputs, stepped or free cutoff, Impedance/Clean
// resonance models, optional Color stage. Stereo, shared coefficients.

#include "Svf.hpp"
#include "Color.hpp"
#include <cstdint>

namespace bigknob {

// The 10 unequally spaced cutoffs of the Altec 9069B (Hz).
constexpr int   kNumSteps = 10;
constexpr float kStepFreqs[kNumSteps] = {
    70.f, 100.f, 150.f, 250.f, 500.f, 1000.f, 2000.f, 3000.f, 5000.f, 7500.f
};

// Nearest step in the log-frequency domain.
inline float quantizeToStep(float freqHz) noexcept {
    float best = kStepFreqs[0];
    float bestDist = 1e30f;
    const float lf = std::log(freqHz);
    for (int i = 0; i < kNumSteps; ++i) {
        const float d = std::fabs(lf - std::log(kStepFreqs[i]));
        if (d < bestDist) { bestDist = d; best = kStepFreqs[i]; }
    }
    return best;
}

// reso 0..1 -> filter Q, exponential 0.5 .. 8.0
inline float resoToQ(float reso) noexcept {
    return 0.5f * std::pow(16.0f, reso);
}

struct BigKnobParams {
    bool  lp        = false;  // false = HP (the real 9069B), true = LP mirror
    float freqHz    = 70.0f;  // 40 .. 10000
    bool  step      = true;   // quantize to kStepFreqs
    float reso      = 0.15f;  // 0..1
    bool  impedance = true;   // true = passive side effects, false = clean Q
    float color     = 0.0f;   // 0..1, 0 = hard bypass of the Color stage
    float gainDb    = 0.0f;   // -12 .. +12
};

} // namespace bigknob
