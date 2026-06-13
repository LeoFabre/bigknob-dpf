#pragma once
// Color.hpp — inductor-style saturation: asymmetric cubic soft clipper with
// loudness compensation and a DC blocker (the asymmetry generates DC).
// The caller skips the stage entirely while Color is exactly 0 — that is the
// bit-exact-bypass guarantee; this file never needs a "0 means transparent"
// special case.

#include <cmath>

namespace bigknob {

// Odd soft clip: ~linear small-signal, saturates to ±2/3. Branchless — clamping
// x into [-1,1] then applying the polynomial is bit-identical to the branched
// form (the poly evaluates to ±2/3 at ±1), and lowers to fminnm/fmaxnm.
inline float colorCubic(float x) noexcept {
    const float xc = std::fmax(-1.0f, std::fmin(1.0f, x));
    return xc - xc * xc * xc * (1.0f / 3.0f);
}

struct ColorCoeffs {
    float pre = 1.0f, post = 1.0f, bias = 0.0f;
    float dcR = 0.9995f;

    // amount: 0..1 (smoothed Color param), drive: >= 1 (Impedance mode scales
    // it with the resonant peak), sr for the DC-blocker pole.
    void set(float amount, float drive, float sr) noexcept {
        pre  = 1.0f + 3.0f * amount * drive;
        bias = 0.12f * amount;
        post = (1.0f + 0.4f * amount) / pre;     // small-signal ~unity + mild makeup
        dcR  = 1.0f - 25.13274f / sr;            // one-pole DC blocker, ~4 Hz
    }
};

struct ColorState {
    float dcX = 0.0f, dcY = 0.0f;

    void reset() noexcept { dcX = dcY = 0.0f; }
    bool finite() const noexcept { return std::isfinite(dcX) && std::isfinite(dcY); }

    // Kept for the unit tests' isolated-clipper checks.
    static inline float cubic(float x) noexcept { return colorCubic(x); }

    inline float process(float in, const ColorCoeffs& c) noexcept {
        // Bias trick: asymmetry (even harmonics) without static DC at rest.
        // BOTH cubics must stay in this function so the compiler emits identical
        // code for them — that's what makes silence-in/silence-out exact even
        // under the DSP TU's -funsafe-math-optimizations.
        const float shaped = colorCubic(in * c.pre + c.bias) - colorCubic(c.bias);
        const float y = shaped - dcX + c.dcR * dcY;   // DC blocker
        dcX = shaped;
        dcY = y;
        return y * c.post;
    }
};

} // namespace bigknob
