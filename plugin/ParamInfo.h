#pragma once

// Parameter indices. Order is the host-facing parameter order.
enum ParamIndex {
    kParamMode = 0,   // 0 = HP, 1 = LP
    kParamFreq,       // Hz, log
    kParamQuantize,   // 0 = Free, 1 = Step
    kParamReso,       // 0..1
    kParamResoMode,   // 0 = Impedance, 1 = Clean
    kParamColor,      // 0..1
    kParamGain,       // dB
    kParamCount
};

struct ParamDescriptor {
    const char* symbol;
    const char* name;
    float min, max, def;
    bool  logarithmic;
    bool  boolean;     // host shows it as a switch
};

static const ParamDescriptor kParams[kParamCount] = {
    { "mode",      "Mode",       0.0f,   1.0f,     0.0f,  false, true  },
    { "freq",      "Frequency",  40.0f,  10000.0f, 70.0f, true,  false },
    { "quantize",  "Free/Step",  0.0f,   1.0f,     1.0f,  false, true  },
    { "reso",      "Resonance",  0.0f,   1.0f,     0.15f, false, false },
    { "reso_mode", "Reso Mode",  0.0f,   1.0f,     0.0f,  false, true  },
    { "color",     "Color",      0.0f,   1.0f,     0.0f,  false, false },
    { "gain",      "Out Gain",  -12.0f,  12.0f,    0.0f,  false, false },
};
