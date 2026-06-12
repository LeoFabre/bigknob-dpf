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

class BigKnobDsp {
public:
    static constexpr uint32_t kCtrlFrames = 32;  // control slice = Bela chunk

    void init(double sampleRate) noexcept {
        sr_    = float(sampleRate);
        aFreq_ = smoothAlpha(0.005f);   // ~5 ms — audible steps, no zipper
        aSlow_ = smoothAlpha(0.020f);   // ~20 ms — reso, color, gain
        aMode_ = smoothAlpha(0.010f);   // ~10 ms — HP<->LP crossfade
        reset();
    }

    void reset() noexcept {
        for (int ch = 0; ch < 2; ++ch) {
            svf_[ch].reset(); opHp_[ch].reset();
            opLp_[ch].reset(); colorSt_[ch].reset();
        }
        // start smoothers on their targets: no sweep at activation
        fcS_    = targetFc();
        resoS_  = p_.reso;
        colorS_ = p_.color;
        modeS_  = p_.lp ? 1.0f : 0.0f;
        gainS_  = dbToLin(p_.gainDb);
        updateCoeffs();
    }

    void setParams(const BigKnobParams& p) noexcept { p_ = p; }

    void process(const float* inL, const float* inR,
                 float* outL, float* outR, uint32_t frames) noexcept {
        uint32_t done = 0;
        while (done < frames) {
            const uint32_t n = (frames - done < kCtrlFrames) ? (frames - done)
                                                             : kCtrlFrames;
            tickControls();
            const bool useColor = colorActive();
            for (uint32_t i = 0; i < n; ++i) {
                outL[done + i] = tickSample(0, inL[done + i], useColor);
                outR[done + i] = tickSample(1, inR[done + i], useColor);
            }
            done += n;
        }
    }

    bool colorActive() const noexcept {
        return colorS_ > 1e-4f || p_.color > 0.0f;
    }

private:
    float smoothAlpha(float seconds) const noexcept {
        return 1.0f - std::exp(-float(kCtrlFrames) / (seconds * sr_));
    }
    static float dbToLin(float db) noexcept { return std::pow(10.0f, db * 0.05f); }

    float targetFc() const noexcept {
        return p_.step ? quantizeToStep(p_.freqHz) : p_.freqHz;
    }

    void tickControls() noexcept {
        fcS_   += aFreq_ * (targetFc() - fcS_);
        resoS_ += aSlow_ * (p_.reso - resoS_);
        gainS_ += aSlow_ * (dbToLin(p_.gainDb) - gainS_);
        modeS_ += aMode_ * ((p_.lp ? 1.0f : 0.0f) - modeS_);
        colorS_ += aSlow_ * (p_.color - colorS_);
        if (p_.color <= 0.0f && colorS_ < 1e-4f)
            colorS_ = 0.0f;                       // snap to true bypass
        updateCoeffs();
        healStates();
    }

    void updateCoeffs() noexcept {
        float fc    = fcS_;
        float q     = resoToQ(resoS_);
        float level = 1.0f;
        float drive = 1.0f;
        if (p_.impedance) {
            // passive-LC side effects: corner shift, damped level loss,
            // colour drive follows the resonant peak
            fc    *= 0.95f + 0.10f * resoS_;
            level  = dbToLin(-1.5f * (1.0f - resoS_));
            drive  = 1.0f + resoS_;
        }
        const float fcMax = 0.45f * sr_;
        if (fc > fcMax) fc = fcMax;
        svfC_.set(fc, q, sr_);
        opC_.set(fc, sr_);
        colorC_.set(colorS_, drive, sr_);
        outGain_ = level * gainS_;
    }

    void healStates() noexcept {
        for (int ch = 0; ch < 2; ++ch) {
            if (!(svf_[ch].finite() && opHp_[ch].finite() &&
                  opLp_[ch].finite() && colorSt_[ch].finite())) {
                svf_[ch].reset(); opHp_[ch].reset();
                opLp_[ch].reset(); colorSt_[ch].reset();
            }
        }
    }

    inline float tickSample(int ch, float in, bool useColor) noexcept {
        float hp, lp;
        svf_[ch].process(in, svfC_, hp, lp);
        // Both 3rd-pole paths always run so their states stay warm and the
        // HP<->LP crossfade is click-free.
        const float hp3 = opHp_[ch].hp(hp, opC_);
        const float lp3 = opLp_[ch].lp(lp, opC_);
        float y = hp3 + modeS_ * (lp3 - hp3);
        if (useColor) y = colorSt_[ch].process(y, colorC_);
        return y * outGain_;
    }

    BigKnobParams p_;
    float sr_    = 48000.0f;
    float aFreq_ = 1.0f, aSlow_ = 1.0f, aMode_ = 1.0f;
    float fcS_ = 70.0f, resoS_ = 0.15f, colorS_ = 0.0f, modeS_ = 0.0f, gainS_ = 1.0f;
    float outGain_ = 1.0f;

    SvfCoeffs     svfC_;
    OnePoleCoeffs opC_;
    ColorCoeffs   colorC_;
    SvfState      svf_[2];
    OnePoleState  opHp_[2], opLp_[2];
    ColorState    colorSt_[2];
};

} // namespace bigknob
