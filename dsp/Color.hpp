#pragma once
// Color.hpp — inductor-style saturation: asymmetric cubic soft clipper with
// loudness compensation and a DC blocker (the asymmetry generates DC).
// The caller skips the stage entirely while Color is exactly 0 — that is the
// bit-exact-bypass guarantee; this file never needs a "0 means transparent"
// special case.

#include <cmath>

namespace bigknob {

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

    // Odd soft clip: ~linear small-signal, saturates to ±2/3.
    static inline float cubic(float x) noexcept {
        if (x >  1.0f) return  2.0f / 3.0f;
        if (x < -1.0f) return -2.0f / 3.0f;
        return x - x * x * x * (1.0f / 3.0f);
    }

    inline float process(float in, const ColorCoeffs& c) noexcept {
        // Bias trick: asymmetry (even harmonics) without static DC at rest.
        const float shaped = cubic(in * c.pre + c.bias) - cubic(c.bias);
        const float y = shaped - dcX + c.dcR * dcY;   // DC blocker
        dcX = shaped;
        dcY = y;
        return y * c.post;
    }
};

} // namespace bigknob
