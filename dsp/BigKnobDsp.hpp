#pragma once
// BigKnobDsp.hpp — BigKnob processing core, host-agnostic (no DPF).
// Altec 9069B model: SVF TPT (2p) -> one-pole TPT (1p) = 18 dB/oct,
// HP/LP crossfaded outputs, stepped or free cutoff, Impedance/Clean
// resonance models, optional Color stage. Stereo, shared coefficients.

#include "Svf.hpp"
#include "Color.hpp"
#include <cstdint>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#define BIGKNOB_NEON 1
#endif

namespace bigknob {

// The 10 unequally spaced cutoffs of the Altec 9069B (Hz).
constexpr int   kNumSteps = 10;
constexpr float kStepFreqs[kNumSteps] = {
    70.f, 100.f, 150.f, 250.f, 500.f, 1000.f, 2000.f, 3000.f, 5000.f, 7500.f
};

// Nearest step in the log-frequency domain. The nearest-in-log-distance test
// |log f - log f_i| < |log f - log f_{i+1}| is exactly f < sqrt(f_i*f_{i+1}),
// so we compare against precomputed geometric-mean boundaries — no libm, and
// the selected (discrete) step is identical to the log-argmin version.
inline float quantizeToStep(float freqHz) noexcept {
    // bounds[i] = sqrt(kStepFreqs[i] * kStepFreqs[i+1]); built once.
    struct Bounds {
        float b[kNumSteps - 1];
        Bounds() noexcept {
            for (int i = 0; i < kNumSteps - 1; ++i)
                b[i] = std::sqrt(kStepFreqs[i] * kStepFreqs[i + 1]);
        }
    };
    static const Bounds t;
    int i = 0;
    while (i < kNumSteps - 1 && freqHz >= t.b[i]) ++i;
    return kStepFreqs[i];
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
        recomputeTargets();
        // start smoothers on their targets: no sweep at activation
        fcS_    = targetFc_;
        resoS_  = p_.reso;
        colorS_ = p_.color;
        modeS_  = p_.lp ? 1.0f : 0.0f;
        gainS_  = targetGain_;
        updateCoeffs();
        // seed the settle-skip trackers so the first tick only rebuilds on motion
        fcApplied_ = fcS_; resoApplied_ = resoS_; colorApplied_ = colorS_;
        gainApplied_ = gainS_; impApplied_ = p_.impedance;
    }

    // NOT thread-safe vs process(): plain struct copy, no atomics. Call from
    // the audio thread only (DPF delivers setParameterValue() there), or
    // ensure external synchronization.
    void setParams(const BigKnobParams& p) noexcept { p_ = p; recomputeTargets(); }

    void process(const float* inL, const float* inR,
                 float* outL, float* outR, uint32_t frames) noexcept {
        uint32_t done = 0;
        while (done < frames) {
            const uint32_t n = (frames - done < kCtrlFrames) ? (frames - done)
                                                             : kCtrlFrames;
            tickControls();
            const bool useColor = colorActive();
            processSlice(inL + done, inR + done, outL + done, outR + done, n, useColor);
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

    // Targets depend only on params, not on time — compute them on param change
    // (rare), not every control tick. Keeps quantizeToStep / pow off the hot path.
    void recomputeTargets() noexcept {
        targetFc_   = p_.step ? quantizeToStep(p_.freqHz) : p_.freqHz;
        targetGain_ = dbToLin(p_.gainDb);
    }

    void tickControls() noexcept {
        fcS_   += aFreq_ * (targetFc_ - fcS_);
        resoS_ += aSlow_ * (p_.reso - resoS_);
        gainS_ += aSlow_ * (targetGain_ - gainS_);
        modeS_ += aMode_ * ((p_.lp ? 1.0f : 0.0f) - modeS_);
        colorS_ += aSlow_ * (p_.color - colorS_);
        if (p_.color <= 0.0f && colorS_ < 1e-4f && colorS_ != 0.0f) {
            colorS_ = 0.0f;                       // snap to true bypass
            colorSt_[0].reset();                  // drop stale DC-blocker state so
            colorSt_[1].reset();                  // reactivation starts clean
        }
        // Settle-skip: the coefficients are a pure function of these inputs, so
        // when none moved this tick the rebuilt coeffs would be bit-identical —
        // skip the 2×tan + pow entirely (the common at-rest case).
        if (fcS_ != fcApplied_ || resoS_ != resoApplied_ || colorS_ != colorApplied_
            || gainS_ != gainApplied_ || p_.impedance != impApplied_) {
            updateCoeffs();
            fcApplied_ = fcS_; resoApplied_ = resoS_; colorApplied_ = colorS_;
            gainApplied_ = gainS_; impApplied_ = p_.impedance;
        }
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
        // When the mode crossfade has fully settled, only one 3rd-pole path
        // reaches the output (the other is weighted by exactly 0/1), so skip
        // the dead one. Bit-exact in steady state; the inactive pole's state
        // just freezes and re-warms over the (smoothed) next mode change.
        float y;
        if (modeS_ == 0.0f) {            // settled HP — LP path dead
            y = opHp_[ch].hp(hp, opC_);
        } else if (modeS_ == 1.0f) {     // settled LP — HP path dead
            y = opLp_[ch].lp(lp, opC_);
        } else {                          // crossfading — both poles run
            const float hp3 = opHp_[ch].hp(hp, opC_);
            const float lp3 = opLp_[ch].lp(lp, opC_);
            y = hp3 + modeS_ * (lp3 - hp3);
        }
        if (useColor) y = colorSt_[ch].process(y, colorC_);
        return y * outGain_;
    }

#if BIGKNOB_NEON
    // 2-lane stereo: both channels share coefficients and run as independent
    // recurrences, so they map onto NEON lanes {L,R}. Op order mirrors the
    // scalar tickSample exactly (no FMA, ftz/contraction aside) — bit-identical.
    void processSlice(const float* inL, const float* inR,
                      float* outL, float* outR, uint32_t n, bool useColor) noexcept {
        // load both channels' states into lanes 0=L, 1=R
        float s_ic1[2] = { svf_[0].ic1, svf_[1].ic1 };
        float s_ic2[2] = { svf_[0].ic2, svf_[1].ic2 };
        float s_hp [2] = { opHp_[0].s,  opHp_[1].s  };
        float s_lp [2] = { opLp_[0].s,  opLp_[1].s  };
        float s_dcX[2] = { colorSt_[0].dcX, colorSt_[1].dcX };
        float s_dcY[2] = { colorSt_[0].dcY, colorSt_[1].dcY };
        float32x2_t ic1 = vld1_f32(s_ic1), ic2 = vld1_f32(s_ic2);
        float32x2_t shp = vld1_f32(s_hp),  slp = vld1_f32(s_lp);
        float32x2_t dcX = vld1_f32(s_dcX), dcY = vld1_f32(s_dcY);

        const float a1 = svfC_.a1, a2 = svfC_.a2, a3 = svfC_.a3, k = svfC_.k;
        const float G  = opC_.G;
        const float mode = modeS_, og = outGain_;
        const float pre = colorC_.pre, bias = colorC_.bias;
        const float dcR = colorC_.dcR, post = colorC_.post;
        const float32x2_t one = vdup_n_f32(1.0f), neg1 = vdup_n_f32(-1.0f);
        const float third = 1.0f / 3.0f;
        // cubic(bias) via the SAME op sequence as the per-sample cubic below, so
        // the two cancel exactly at silence (matches scalar ColorState::process).
        const float32x2_t biasV = vdup_n_f32(bias);
        const float32x2_t bc    = vminnm_f32(one, vmaxnm_f32(neg1, biasV));
        const float32x2_t biasOut = vsub_f32(bc, vmul_n_f32(vmul_f32(vmul_f32(bc, bc), bc), third));

        // Settled-mode regime, constant across the slice → the invariant branch
        // below is loop-unswitched by the compiler into HP-only / LP-only / mix
        // loops; the settled cases skip the dead 3rd pole entirely.
        const int modeReg = (mode == 0.0f) ? 0 : (mode == 1.0f) ? 1 : 2;
        for (uint32_t i = 0; i < n; ++i) {
            float32x2_t in = { inL[i], inR[i] };
            // SVF (TPT). FMA-fused (vfma_n: a + b*scalar) — matches the scalar TU
            // built with -funsafe-math-optimizations, so both paths agree.
            float32x2_t v3 = vsub_f32(in, ic2);
            float32x2_t v1 = vfma_n_f32(vmul_n_f32(v3, a2), ic1, a1);          // a2*v3 + a1*ic1
            float32x2_t v2 = vfma_n_f32(vfma_n_f32(ic2, ic1, a2), v3, a3);     // ic2 + a2*ic1 + a3*v3
            ic1 = vsub_f32(vmul_n_f32(v1, 2.0f), ic1);
            ic2 = vsub_f32(vmul_n_f32(v2, 2.0f), ic2);
            float32x2_t hp = vsub_f32(vfms_n_f32(in, v1, k), v2);             // (in - k*v1) - v2
            float32x2_t lp = v2;
            float32x2_t y;
            if (modeReg == 0) {                 // settled HP — skip the LP pole
                float32x2_t dh = vsub_f32(hp, shp);
                float32x2_t yh = vfma_n_f32(shp, dh, G);
                shp = vfma_n_f32(yh, dh, G);
                y = vsub_f32(hp, yh);
            } else if (modeReg == 1) {          // settled LP — skip the HP pole
                float32x2_t dl = vsub_f32(lp, slp);
                float32x2_t yl = vfma_n_f32(slp, dl, G);
                slp = vfma_n_f32(yl, dl, G);
                y = yl;
            } else {                             // crossfading — both poles run
                float32x2_t dh = vsub_f32(hp, shp);
                float32x2_t yh = vfma_n_f32(shp, dh, G);
                shp = vfma_n_f32(yh, dh, G);
                float32x2_t hp3 = vsub_f32(hp, yh);
                float32x2_t dl = vsub_f32(lp, slp);
                float32x2_t yl = vfma_n_f32(slp, dl, G);
                slp = vfma_n_f32(yl, dl, G);
                y = vfma_n_f32(hp3, vsub_f32(yl, hp3), mode);
            }
            if (useColor) {
                // shaped = colorCubic(y*pre + bias) - biasOut
                float32x2_t xx = vfma_n_f32(vdup_n_f32(bias), y, pre);
                float32x2_t xc = vminnm_f32(one, vmaxnm_f32(neg1, xx));
                float32x2_t cube = vmul_n_f32(vmul_f32(vmul_f32(xc, xc), xc), third);
                float32x2_t shaped = vsub_f32(vsub_f32(xc, cube), biasOut);
                // DC blocker: yc = (shaped - dcX) + dcR*dcY
                float32x2_t yc = vfma_n_f32(vsub_f32(shaped, dcX), dcY, dcR);
                dcX = shaped; dcY = yc;
                y = vmul_n_f32(yc, post);
            }
            y = vmul_n_f32(y, og);
            vst1_lane_f32(&outL[i], y, 0);
            vst1_lane_f32(&outR[i], y, 1);
        }
        // store states back
        vst1_f32(s_ic1, ic1); vst1_f32(s_ic2, ic2);
        vst1_f32(s_hp, shp);  vst1_f32(s_lp, slp);
        vst1_f32(s_dcX, dcX); vst1_f32(s_dcY, dcY);
        svf_[0].ic1 = s_ic1[0]; svf_[1].ic1 = s_ic1[1];
        svf_[0].ic2 = s_ic2[0]; svf_[1].ic2 = s_ic2[1];
        opHp_[0].s  = s_hp[0];  opHp_[1].s  = s_hp[1];
        opLp_[0].s  = s_lp[0];  opLp_[1].s  = s_lp[1];
        colorSt_[0].dcX = s_dcX[0]; colorSt_[1].dcX = s_dcX[1];
        colorSt_[0].dcY = s_dcY[0]; colorSt_[1].dcY = s_dcY[1];
    }
#else
    void processSlice(const float* inL, const float* inR,
                      float* outL, float* outR, uint32_t n, bool useColor) noexcept {
        for (uint32_t i = 0; i < n; ++i) {
            outL[i] = tickSample(0, inL[i], useColor);
            outR[i] = tickSample(1, inR[i], useColor);
        }
    }
#endif

    BigKnobParams p_;
    float sr_    = 48000.0f;
    float aFreq_ = 1.0f, aSlow_ = 1.0f, aMode_ = 1.0f;
    float fcS_ = 70.0f, resoS_ = 0.15f, colorS_ = 0.0f, modeS_ = 0.0f, gainS_ = 1.0f;
    float outGain_ = 1.0f;
    float targetFc_ = 70.0f, targetGain_ = 1.0f;
    // settle-skip trackers: last smoother values that coeffs were built from
    float fcApplied_ = 1e30f, resoApplied_ = 1e30f, colorApplied_ = 1e30f, gainApplied_ = 1e30f;
    bool  impApplied_ = false;

    SvfCoeffs     svfC_;
    OnePoleCoeffs opC_;
    ColorCoeffs   colorC_;
    SvfState      svf_[2];
    OnePoleState  opHp_[2], opLp_[2];
    ColorState    colorSt_[2];
};

} // namespace bigknob
