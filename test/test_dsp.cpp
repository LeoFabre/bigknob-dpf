// BigKnob DSP tests — standalone, no DPF required.
// Build: clang++ -std=c++17 -O2 -I../dsp test_dsp.cpp -o /tmp/test_bigknob && /tmp/test_bigknob
#include "Svf.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>

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

int main() {
    test_svf_hp_slope_12db();
    test_svf_lp_slope_12db();
    test_onepole_slope_6db();
    test_svf_resonance_peaks();
    test_svf_state_finite_guard();
    std::printf("ALL DSP TESTS PASSED\n");
    return 0;
}
