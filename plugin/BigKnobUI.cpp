#include "DistrhoUI.hpp"
#include "ParamInfo.h"
#include <cmath>
#include <cstdio>

START_NAMESPACE_DISTRHO

using DGL_NAMESPACE::Color;

namespace {

constexpr float kKnobY = 92.0f, kKnobR = 28.0f;
constexpr float kSwitchY = 188.0f, kSwitchW = 76.0f, kSwitchH = 26.0f;

constexpr float    kKnobX[4]       = { 70.0f, 180.0f, 290.0f, 400.0f };
constexpr uint32_t kKnobParam[4]   = { kParamFreq, kParamReso, kParamColor, kParamGain };
const char*        kKnobLabel[4]   = { "FREQ", "RESO", "COLOR", "GAIN" };

constexpr float    kSwitchX[3]     = { 70.0f, 235.0f, 400.0f };
constexpr uint32_t kSwitchParam[3] = { kParamMode, kParamQuantize, kParamResoMode };
const char*        kSwitchLab[3][2] = { {"HP", "LP"}, {"FREE", "STEP"}, {"IMP", "CLEAN"} };

float toNorm(uint32_t p, float v) {
    const ParamDescriptor& d = kParams[p];
    if (d.logarithmic)
        return std::log(v / d.min) / std::log(d.max / d.min);
    return (v - d.min) / (d.max - d.min);
}

float fromNorm(uint32_t p, float n) {
    const ParamDescriptor& d = kParams[p];
    n = n < 0.0f ? 0.0f : (n > 1.0f ? 1.0f : n);
    if (d.logarithmic)
        return d.min * std::pow(d.max / d.min, n);
    return d.min + n * (d.max - d.min);
}

} // namespace

class BigKnobUI : public UI {
public:
    BigKnobUI() : UI(DISTRHO_UI_DEFAULT_WIDTH, DISTRHO_UI_DEFAULT_HEIGHT) {
        loadSharedResources();   // registers DPF's bundled font for text()
        for (uint32_t i = 0; i < kParamCount; ++i)
            fValues[i] = kParams[i].def;
    }

protected:
    void parameterChanged(uint32_t index, float value) override {
        fValues[index] = value;
        repaint();
    }

    void onNanoDisplay() override {
        beginPath(); rect(0, 0, getWidth(), getHeight());
        fillColor(Color(24, 24, 28)); fill();

        fontSize(18.0f);
        textAlign(ALIGN_CENTER | ALIGN_TOP);
        fillColor(Color(230, 220, 200));
        text(getWidth() * 0.5f, 8.0f, "BigKnob", nullptr);

        for (int i = 0; i < 4; ++i) drawKnob(i);
        for (int i = 0; i < 3; ++i) drawSwitch(i);
    }

    bool onMouse(const MouseEvent& ev) override {
        if (ev.button != 1) return false;
        if (ev.press) {
            for (int i = 0; i < 3; ++i) {
                if (hitSwitch(i, float(ev.pos.getX()), float(ev.pos.getY()))) {
                    const uint32_t p = kSwitchParam[i];
                    const float v = fValues[p] > 0.5f ? 0.0f : 1.0f;
                    fValues[p] = v;
                    setParameterValue(p, v);
                    repaint();
                    return true;
                }
            }
            for (int i = 0; i < 4; ++i) {
                if (hitKnob(i, float(ev.pos.getX()), float(ev.pos.getY()))) {
                    fDragKnob  = i;
                    fDragY     = float(ev.pos.getY());
                    fDragStart = toNorm(kKnobParam[i], fValues[kKnobParam[i]]);
                    return true;
                }
            }
        } else if (fDragKnob >= 0) {
            fDragKnob = -1;
            return true;
        }
        return false;
    }

    bool onMotion(const MotionEvent& ev) override {
        if (fDragKnob < 0) return false;
        const uint32_t p = kKnobParam[fDragKnob];
        const float n = fDragStart + (fDragY - float(ev.pos.getY())) / 200.0f;
        const float val = fromNorm(p, n);
        fValues[p] = val;
        setParameterValue(p, val);
        repaint();
        return true;
    }

private:
    void drawKnob(int i) {
        const uint32_t p = kKnobParam[i];
        const float x = kKnobX[i], y = kKnobY;
        const float n = toNorm(p, fValues[p]);
        // 270° arc, from 7:30 to 4:30
        const float a0 = 0.75f * kPi, a1 = a0 + 1.5f * kPi;
        beginPath(); arc(x, y, kKnobR, a0, a1, NanoVG::CW);
        strokeWidth(4.0f); strokeColor(Color(60, 60, 66)); stroke();
        beginPath(); arc(x, y, kKnobR, a0, a0 + n * 1.5f * kPi, NanoVG::CW);
        strokeColor(Color(235, 170, 60)); stroke();

        char buf[32];
        const float fval = fValues[p];
        if (p == kParamFreq) {
            if (fval >= 1000.0f)
                std::snprintf(buf, sizeof buf, "%.1fk", fval / 1000.0f);
            else
                std::snprintf(buf, sizeof buf, "%.0f", fval);
        } else if (p == kParamGain) {
            std::snprintf(buf, sizeof buf, "%+.1f", fval);
        } else {
            std::snprintf(buf, sizeof buf, "%.0f%%", fval * 100.0f);
        }

        fontSize(13.0f);
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        fillColor(Color(230, 220, 200));
        text(x, y, buf, nullptr);
        fontSize(12.0f);
        fillColor(Color(150, 150, 155));
        text(x, y + kKnobR + 14.0f, kKnobLabel[i], nullptr);
    }

    void drawSwitch(int i) {
        const float x = kSwitchX[i] - kSwitchW * 0.5f, y = kSwitchY;
        const bool on = fValues[kSwitchParam[i]] > 0.5f;
        beginPath(); roundedRect(x, y, kSwitchW, kSwitchH, 6.0f);
        fillColor(Color(45, 45, 52)); fill();
        beginPath();
        roundedRect(on ? x + kSwitchW * 0.5f : x, y, kSwitchW * 0.5f, kSwitchH, 6.0f);
        fillColor(Color(235, 170, 60)); fill();
        fontSize(11.0f);
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        fillColor(on ? Color(150, 150, 155) : Color(20, 20, 22));
        text(x + kSwitchW * 0.25f, y + kSwitchH * 0.5f, kSwitchLab[i][0], nullptr);
        fillColor(on ? Color(20, 20, 22) : Color(150, 150, 155));
        text(x + kSwitchW * 0.75f, y + kSwitchH * 0.5f, kSwitchLab[i][1], nullptr);
    }

    bool hitKnob(int i, float px, float py) const {
        const float dx = px - kKnobX[i], dy = py - kKnobY;
        return dx * dx + dy * dy <= (kKnobR + 8.0f) * (kKnobR + 8.0f);
    }

    bool hitSwitch(int i, float px, float py) const {
        const float x = kSwitchX[i] - kSwitchW * 0.5f;
        return px >= x && px <= x + kSwitchW && py >= kSwitchY && py <= kSwitchY + kSwitchH;
    }

    static constexpr float kPi = 3.14159265358979323846f;

    float fValues[kParamCount];
    int   fDragKnob  = -1;
    float fDragY     = 0.0f;
    float fDragStart = 0.0f;

    DISTRHO_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(BigKnobUI)
};

UI* createUI() { return new BigKnobUI(); }

END_NAMESPACE_DISTRHO
