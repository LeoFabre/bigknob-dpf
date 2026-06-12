#include "DistrhoPlugin.hpp"
#include "ParamInfo.h"
#include "BigKnobDsp.hpp"
#include "DenormalGuard.hpp"

START_NAMESPACE_DISTRHO

class BigKnobPlugin : public Plugin {
public:
    BigKnobPlugin() : Plugin(kParamCount, 0, 0) {
        for (uint32_t i = 0; i < kParamCount; ++i)
            fParams[i] = kParams[i].def;
        fDsp.init(getSampleRate());
        applyParams();
    }

protected:
    const char* getLabel()       const override { return "BigKnob"; }
    const char* getDescription() const override { return "King Tubby style stepped HP/LP filter (Altec 9069B model)."; }
    const char* getMaker()       const override { return "Dubplex"; }
    const char* getHomePage()    const override { return DISTRHO_PLUGIN_URI; }
    const char* getLicense()     const override { return "GPL-3.0-or-later"; }
    uint32_t    getVersion()     const override { return d_version(0, 1, 0); }
    int64_t     getUniqueId()    const override { return d_cconst('D', 'b', 'B', 'K'); }

    void initParameter(uint32_t index, Parameter& p) override {
        const ParamDescriptor& d = kParams[index];
        p.hints = kParameterIsAutomatable
                | (d.logarithmic ? kParameterIsLogarithmic : 0x0u)
                | (d.boolean ? (kParameterIsBoolean | kParameterIsInteger) : 0x0u);
        p.name       = d.name;
        p.symbol     = d.symbol;
        p.ranges.def = d.def;
        p.ranges.min = d.min;
        p.ranges.max = d.max;
    }

    float getParameterValue(uint32_t index) const override { return fParams[index]; }

    void setParameterValue(uint32_t index, float value) override {
        fParams[index] = value;
        applyParams();
    }

    void activate() override {
        fDsp.init(getSampleRate());
        applyParams();
    }

    void sampleRateChanged(double) override {
        fDsp.init(getSampleRate());
        applyParams();
    }

    void run(const float** inputs, float** outputs, uint32_t frames) override {
        ftz::armOnce();
        fDsp.process(inputs[0], inputs[1], outputs[0], outputs[1], frames);
    }

private:
    void applyParams() noexcept {
        bigknob::BigKnobParams p;
        p.lp        = fParams[kParamMode]     > 0.5f;
        p.freqHz    = fParams[kParamFreq];
        p.step      = fParams[kParamQuantize] > 0.5f;
        p.reso      = fParams[kParamReso];
        p.impedance = fParams[kParamResoMode] < 0.5f;
        p.color     = fParams[kParamColor];
        p.gainDb    = fParams[kParamGain];
        fDsp.setParams(p);
    }

    float               fParams[kParamCount];
    bigknob::BigKnobDsp fDsp;
    DISTRHO_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(BigKnobPlugin)
};

Plugin* createPlugin() { return new BigKnobPlugin(); }

END_NAMESPACE_DISTRHO
