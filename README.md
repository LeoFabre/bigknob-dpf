# bigknob-dpf

**BigKnob** — King Tubby style stepped HP/LP filter, modeled on the **Altec
9069B** (the passive LC filter in Tubby's MCI console). DPF, VST3, Dubplex.

## Parameters

| Symbol      | Name      | Range            | Default | Notes |
|-------------|-----------|------------------|---------|-------|
| `mode`      | Mode      | HP / LP          | HP      | LP is a mirror extension |
| `freq`      | Frequency | 40 Hz–10 kHz log | 70 Hz   | quantized in Step mode |
| `quantize`  | Free/Step | switch           | Step    | steps: 70, 100, 150, 250, 500, 1k, 2k, 3k, 5k, 7.5k |
| `reso`      | Resonance | 0–100 %          | 15 %    | Q 0.5→8 |
| `reso_mode` | Reso Mode | Impedance/Clean  | Impedance | Impedance adds passive-LC side effects |
| `color`     | Color     | 0–100 %          | 0 %     | inductor-style saturation, true bypass at 0 |
| `gain`      | Out Gain  | −12…+12 dB       | 0 dB    | |

## Build

    cmake -B build -DCMAKE_BUILD_TYPE=Release        # UI on (desktop dev)
    cmake -B build -DBIGKNOB_BUILD_UI=OFF ...        # headless (Bela/sushi)
    cmake --build build -j8                           # -> build/bin/BigKnob.vst3

Cross-build for the Bela from nexus-preamp:
`./sushi-on-bela/cross-build-dpf-plugin.sh plugins/bigknob-dpf` (headless by default).

## DSP tests (no DPF needed)

    clang++ -std=c++17 -O2 -Idsp test/test_dsp.cpp -o /tmp/test_bigknob && /tmp/test_bigknob

DSP: TPT SVF (2p) + TPT one-pole = 18 dB/oct, simultaneous HP/LP with a 10 ms
crossfade, 5 ms cutoff smoothing (audible steps, no zipper), NaN self-healing
states, FTZ on aarch64.
