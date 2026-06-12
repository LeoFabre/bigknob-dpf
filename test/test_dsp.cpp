// BigKnob DSP tests — standalone, no DPF required.
// Build: clang++ -std=c++17 -O2 -I../dsp test_dsp.cpp -o /tmp/test_bigknob && /tmp/test_bigknob
#include "Svf.hpp"
#include "Color.hpp"
#include "BigKnobDsp.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace bigknob;

static constexpr double kSr = 48000.0;
static constexpr double kPiD = 3.14159265358979323846;

// Steady-state magnitude |H(f)| of any mono sample-processor lambda.
// Feeds a unit sine, discards 0.1 s, measures RMS over 0.2 s.
template <typename Proc>
static double magAt(Proc&& proc, double f, double amp = 1.0) {
    const int settle = int(kSr * 0.1), measure = int(kSr * 0.2);
    double phase = 0.0;
    const double w = 2.0 * kPiD * f / kSr;
    for (int i = 0; i < settle; ++i) { (void) proc(float(amp * std::sin(phase))); phase += w; }
    double acc = 0.0;
    for (int i = 0; i < measure; ++i) {
        const float y = proc(float(amp * std::sin(phase))); phase += w;
        acc += double(y) * y;
    }
    return std::sqrt(acc / measure) * std::sqrt(2.0) / amp;
}

static double db(double x) { return 20.0 * std::log10(x); }

static void test_svf_hp_slope_12db() {
    SvfCoeffs c; c.set(1000.0f, 0.7071f, float(kSr));
    SvfState s;
    auto hpProc = [&](float in) { float hp, lp; s.process(in, c, hp, lp); return hp; };
    s.reset();
    const double m250 = magAt(hpProc, 250.0);
    s.reset();
    const double m125 = magAt(hpProc, 125.0);
    const double slope = db(m250) - db(m125);          // one octave apart
    assert(slope > 10.5 && slope < 13.5);              // 2-pole: ~12 dB/oct
    s.reset();
    assert(std::fabs(db(magAt(hpProc, 8000.0))) < 1.0); // passband flat
}

static void test_svf_lp_slope_12db() {
    SvfCoeffs c; c.set(500.0f, 0.7071f, float(kSr));
    SvfState s;
    auto lpProc = [&](float in) { float hp, lp; s.process(in, c, hp, lp); return lp; };
    s.reset();
    const double m2k = magAt(lpProc, 2000.0);
    s.reset();
    const double m4k = magAt(lpProc, 4000.0);
    const double slope = db(m2k) - db(m4k);
    assert(slope > 10.5 && slope < 13.5);              // 2-pole: ~12 dB/oct
    s.reset();
    assert(std::fabs(db(magAt(lpProc, 50.0))) < 1.0);
}

static void test_onepole_slope_6db() {
    OnePoleCoeffs c; c.set(1000.0f, float(kSr));
    OnePoleState s;
    auto hpProc = [&](float in) { return s.hp(in, c); };
    s.reset();
    const double m250 = magAt(hpProc, 250.0);
    s.reset();
    const double m125 = magAt(hpProc, 125.0);
    const double slope = db(m250) - db(m125);
    assert(slope > 5.0 && slope < 7.0);
}

static void test_svf_resonance_peaks() {
    // High Q must show a peak at fc clearly above unity.
    SvfCoeffs c; c.set(1000.0f, 8.0f, float(kSr));
    SvfState s;
    auto hpProc = [&](float in) { float hp, lp; s.process(in, c, hp, lp); return hp; };
    s.reset();
    assert(db(magAt(hpProc, 1000.0)) > 16.0);          // Q=8 -> ~+18 dB peak at fc
}

static void test_svf_state_finite_guard() {
    SvfCoeffs c; c.set(1000.0f, 1.0f, float(kSr));
    SvfState s;
    float hp, lp;
    s.process(std::nanf(""), c, hp, lp);
    assert(!s.finite());                                // guard detects poisoning
    s.reset();
    assert(s.finite());
}

static void test_color_silence_in_silence_out() {
    ColorCoeffs c; c.set(1.0f, 2.0f, float(kSr));
    ColorState s;
    for (int i = 0; i < 1000; ++i)
        assert(s.process(0.0f, c) == 0.0f);   // bias trick: zero in -> zero out, exactly
}

static void test_color_small_signal_gain() {
    // Small-signal gain ~= pre * post * (1 - bias^2)  (cubic'(bias) = 1 - bias^2)
    ColorCoeffs c; c.set(0.5f, 1.0f, float(kSr));
    ColorState s;
    auto proc = [&](float in) { return s.process(in, c); };
    const double expected = double(c.pre) * c.post * (1.0 - double(c.bias) * c.bias);
    const double measured = magAt(proc, 1000.0, 0.001);  // tiny amplitude: linear region
    assert(std::fabs(measured - expected) / expected < 0.05);
}

static void test_color_bounded_output() {
    ColorCoeffs c; c.set(1.0f, 2.0f, float(kSr));
    ColorState s;
    for (int i = 0; i < 4800; ++i) {
        const float wild = (i % 2 ? 10.0f : -10.0f);
        const float y = s.process(wild, c);
        assert(std::isfinite(y) && std::fabs(y) < 1.5f);
    }
}

static void test_color_no_dc_after_settle() {
    // Heavily asymmetric drive on a sine generates DC -> the blocker must kill it.
    ColorCoeffs c; c.set(1.0f, 2.0f, float(kSr));
    ColorState s;
    double phase = 0.0, w = 2.0 * kPiD * 100.0 / kSr;
    for (int i = 0; i < 48000; ++i) { (void) s.process(float(std::sin(phase)), c); phase += w; }
    double mean = 0.0;
    const int n = 9600;  // 20 full cycles of 100 Hz
    for (int i = 0; i < n; ++i) { mean += s.process(float(std::sin(phase)), c); phase += w; }
    mean /= n;
    assert(std::fabs(mean) < 0.005);
}

static void test_step_table_is_the_altec_9069b() {
    static const float altec[10] = {70,100,150,250,500,1000,2000,3000,5000,7500};
    static_assert(kNumSteps == 10, "9069B has 10 positions");
    for (int i = 0; i < 10; ++i) assert(kStepFreqs[i] == altec[i]);
}

static void test_quantize_lands_on_steps() {
    for (int i = 0; i < kNumSteps; ++i)
        assert(quantizeToStep(kStepFreqs[i]) == kStepFreqs[i]);   // fixed points
    // Nearest in LOG domain: log-midpoint of 70..100 is sqrt(7000) ~= 83.67
    assert(quantizeToStep(80.0f)  == 70.0f);
    assert(quantizeToStep(90.0f)  == 100.0f);
    assert(quantizeToStep(40.0f)  == 70.0f);      // below range clamps to first
    assert(quantizeToStep(10000.0f) == 7500.0f);  // above range clamps to last
}

static void test_reso_to_q_range() {
    assert(std::fabs(resoToQ(0.0f) - 0.5f) < 1e-4f);
    assert(std::fabs(resoToQ(1.0f) - 8.0f) < 1e-3f);
    assert(resoToQ(0.5f) > resoToQ(0.25f));        // monotonic
}

// Buffer-based |H(f)| probe for the full BigKnobDsp (block API).
static double dspMagAt(bigknob::BigKnobDsp& dsp, double f) {
    const int settle = int(kSr * 0.1), measure = int(kSr * 0.2);
    std::vector<float> in(settle + measure), oL(settle + measure), oR(settle + measure);
    double phase = 0.0; const double w = 2.0 * kPiD * f / kSr;
    for (int i = 0; i < settle + measure; ++i) { in[i] = float(std::sin(phase)); phase += w; }
    dsp.process(in.data(), in.data(), oL.data(), oR.data(), uint32_t(in.size()));
    double acc = 0.0;
    for (int i = settle; i < settle + measure; ++i) acc += double(oL[i]) * oL[i];
    return std::sqrt(acc / measure) * std::sqrt(2.0);
}

// reso 0.25 -> Q = 1.0: SVF(Q=1) x one-pole = exact 3rd-order Butterworth.
static bigknob::BigKnobParams butterworthParams(float fc, bool lpMode) {
    bigknob::BigKnobParams p;
    p.lp = lpMode; p.freqHz = fc; p.step = false;
    p.reso = 0.25f; p.impedance = false; p.color = 0.0f; p.gainDb = 0.0f;
    return p;
}

static void test_chain_hp_18db_per_oct() {
    bigknob::BigKnobDsp dsp;
    dsp.init(kSr);
    dsp.setParams(butterworthParams(1000.0f, false));
    dsp.reset();
    const double m250 = dspMagAt(dsp, 250.0);
    dsp.reset();
    const double m125 = dspMagAt(dsp, 125.0);
    const double slope = db(m250) - db(m125);
    assert(slope > 16.5 && slope < 19.5);              // 3 poles: ~18 dB/oct
    dsp.reset();
    assert(std::fabs(db(dspMagAt(dsp, 8000.0))) < 1.0); // passband flat
}

static void test_chain_lp_mirror_18db_per_oct() {
    bigknob::BigKnobDsp dsp;
    dsp.init(kSr);
    dsp.setParams(butterworthParams(500.0f, true));
    dsp.reset();
    const double m2k = dspMagAt(dsp, 2000.0);
    dsp.reset();
    const double m4k = dspMagAt(dsp, 4000.0);
    const double slope = db(m2k) - db(m4k);
    assert(slope > 17.0 && slope < 20.5);              // warp inflates slightly
    dsp.reset();
    assert(std::fabs(db(dspMagAt(dsp, 50.0))) < 1.0);
}

static void test_step_mode_quantizes_the_cutoff() {
    // freqHz=80 in Step mode must behave as fc=70: compare against a Free-mode
    // instance pinned at exactly 70 Hz.
    bigknob::BigKnobDsp stepped, pinned;
    stepped.init(kSr); pinned.init(kSr);
    auto p = butterworthParams(80.0f, false);
    p.step = true;  stepped.setParams(p); stepped.reset();
    auto q = butterworthParams(70.0f, false);
    pinned.setParams(q); pinned.reset();
    const double a = dspMagAt(stepped, 35.0);
    const double b = dspMagAt(pinned, 35.0);
    assert(std::fabs(db(a) - db(b)) < 0.1);
}

static void test_stability_under_brutal_jumps() {
    // Max reso, impedance + color on, cutoff slammed 40<->10000 every 64 samples.
    bigknob::BigKnobDsp dsp;
    dsp.init(kSr);
    bigknob::BigKnobParams p;
    p.step = false; p.reso = 1.0f; p.impedance = true; p.color = 1.0f;
    uint32_t lcg = 1;
    std::vector<float> in(64), oL(64), oR(64);
    for (int blk = 0; blk < 1500; ++blk) {           // 2 s total
        p.freqHz = (blk % 2) ? 10000.0f : 40.0f;
        dsp.setParams(p);
        for (int i = 0; i < 64; ++i) {
            lcg = lcg * 1664525u + 1013904223u;       // deterministic noise
            in[i] = (float(lcg >> 8) / 8388608.0f) - 1.0f;
        }
        dsp.process(in.data(), in.data(), oL.data(), oR.data(), 64);
        for (int i = 0; i < 64; ++i) {
            assert(std::isfinite(oL[i]) && std::isfinite(oR[i]));
            assert(std::fabs(oL[i]) < 20.0f);
        }
    }
}

static void test_color_zero_is_bit_exact_bypass() {
    // A: color stays 0 forever. B: color active on REAL SIGNAL (builds up
    // non-zero ColorState), then back to 0. After B's smoother snaps, both
    // must produce IDENTICAL output bits — proving the stage is truly skipped
    // (a broken bypass would run B's stale DC blocker and diverge).
    bigknob::BigKnobDsp a, b;
    a.init(kSr); b.init(kSr);
    auto p = butterworthParams(1000.0f, false);
    a.setParams(p); a.reset();
    auto pb = p; pb.color = 0.5f;
    b.setParams(pb); b.reset();

    const int n1 = 48000;
    std::vector<float> sig(n1), t1(n1), t2(n1);
    double phase = 0.0; const double w = 2.0 * kPiD * 220.0 / kSr;
    for (int i = 0; i < n1; ++i) { sig[i] = 0.8f * float(std::sin(phase)); phase += w; }

    b.process(sig.data(), sig.data(), t1.data(), t2.data(), n1);  // color UP on signal
    pb.color = 0.0f; b.setParams(pb);
    b.process(sig.data(), sig.data(), t1.data(), t2.data(), n1);  // fade out + snap, still on signal
    a.process(sig.data(), sig.data(), t1.data(), t2.data(), n1);  // same signal history for A
    a.process(sig.data(), sig.data(), t1.data(), t2.data(), n1);
    assert(!b.colorActive());

    // identical noise through both — must match bit-for-bit
    uint32_t lcg = 7;
    std::vector<float> in(4800), aL(4800), aR(4800), bL(4800), bR(4800);
    for (int i = 0; i < 4800; ++i) {
        lcg = lcg * 1664525u + 1013904223u;
        in[i] = (float(lcg >> 8) / 8388608.0f) - 1.0f;
    }
    a.process(in.data(), in.data(), aL.data(), aR.data(), 4800);
    b.process(in.data(), in.data(), bL.data(), bR.data(), 4800);
    assert(std::memcmp(aL.data(), bL.data(), 4800 * sizeof(float)) == 0);
}

static void test_nan_input_self_heals() {
    bigknob::BigKnobDsp dsp;
    dsp.init(kSr);
    dsp.setParams(butterworthParams(1000.0f, false));
    dsp.reset();
    std::vector<float> in(256, 0.5f), oL(256), oR(256);
    in[0] = std::nanf("");
    dsp.process(in.data(), in.data(), oL.data(), oR.data(), 256);
    // next buffer must be fully finite (states healed at slice boundaries)
    std::fill(in.begin(), in.end(), 0.5f);
    dsp.process(in.data(), in.data(), oL.data(), oR.data(), 256);
    for (int i = 0; i < 256; ++i) assert(std::isfinite(oL[i]));
}

static void test_impedance_level_side_effect() {
    // At reso=0, Impedance mode loses ~1.5 dB vs Clean (damped passive network).
    bigknob::BigKnobDsp clean, imp;
    clean.init(kSr); imp.init(kSr);
    auto p = butterworthParams(100.0f, false);
    p.reso = 0.0f;
    clean.setParams(p); clean.reset();
    auto pi = p; pi.impedance = true;
    imp.setParams(pi); imp.reset();
    const double diff = db(dspMagAt(clean, 5000.0)) - db(dspMagAt(imp, 5000.0));
    assert(diff > 1.0 && diff < 2.0);
}

int main() {
    test_svf_hp_slope_12db();
    test_svf_lp_slope_12db();
    test_onepole_slope_6db();
    test_svf_resonance_peaks();
    test_svf_state_finite_guard();
    test_color_silence_in_silence_out();
    test_color_small_signal_gain();
    test_color_bounded_output();
    test_color_no_dc_after_settle();
    test_step_table_is_the_altec_9069b();
    test_quantize_lands_on_steps();
    test_reso_to_q_range();
    test_chain_hp_18db_per_oct();
    test_chain_lp_mirror_18db_per_oct();
    test_step_mode_quantizes_the_cutoff();
    test_stability_under_brutal_jumps();
    test_color_zero_is_bit_exact_bypass();
    test_nan_input_self_heals();
    test_impedance_level_side_effect();
    std::printf("ALL DSP TESTS PASSED\n");
    return 0;
}
