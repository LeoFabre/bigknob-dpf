#pragma once
// Svf.hpp — TPT (topology-preserving transform) filters after Zavalishin,
// "The Art of VA Filter Design". Coefficients are shared across channels and
// set at control rate only; states are per-channel.

#include <cmath>

namespace bigknob {

constexpr float kPi = 3.14159265358979323846f;

// 2-pole state-variable filter.
struct SvfCoeffs {
    float k = 1.0f;                          // damping = 1/Q
    float a1 = 0.0f, a2 = 0.0f, a3 = 0.0f;

    // fc: cutoff Hz (caller clamps to < 0.45*sr), q in [0.1, 16]
    void set(float fc, float q, float sr) noexcept {
        const float g = std::tan(kPi * fc / sr);
        k  = 1.0f / q;
        a1 = 1.0f / (1.0f + g * (g + k));
        a2 = g * a1;
        a3 = g * a2;
    }
};

struct SvfState {
    float ic1 = 0.0f, ic2 = 0.0f;

    void reset() noexcept { ic1 = ic2 = 0.0f; }
    bool finite() const noexcept { return std::isfinite(ic1) && std::isfinite(ic2); }

    // One sample; writes the simultaneous HP and LP outputs.
    inline void process(float in, const SvfCoeffs& c, float& hp, float& lp) noexcept {
        const float v3 = in - ic2;
        const float v1 = c.a1 * ic1 + c.a2 * v3;
        const float v2 = ic2 + c.a2 * ic1 + c.a3 * v3;
        ic1 = 2.0f * v1 - ic1;
        ic2 = 2.0f * v2 - ic2;
        hp  = in - c.k * v1 - v2;
        lp  = v2;
    }
};

// 1-pole TPT — the third pole for the 9069B's 18 dB/oct.
struct OnePoleCoeffs {
    float G = 0.0f;
    void set(float fc, float sr) noexcept {
        const float g = std::tan(kPi * fc / sr);
        G = g / (1.0f + g);
    }
};

struct OnePoleState {
    float s = 0.0f;

    void reset() noexcept { s = 0.0f; }
    bool finite() const noexcept { return std::isfinite(s); }

    inline float lp(float in, const OnePoleCoeffs& c) noexcept {
        const float v = c.G * (in - s);
        const float y = v + s;
        s = y + v;
        return y;
    }
    inline float hp(float in, const OnePoleCoeffs& c) noexcept {
        return in - lp(in, c);
    }
};

} // namespace bigknob
