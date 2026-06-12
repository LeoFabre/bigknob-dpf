cmake_minimum_required(VERSION 3.22)
project(BigKnob VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BIGKNOB_BUILD_UI    "Build the optional NanoVG UI" ON)
option(BIGKNOB_BUILD_TESTS "Build offline DSP tests"      ON)

add_subdirectory(dpf)

if(BIGKNOB_BUILD_UI)
    dpf_add_plugin(BigKnob
        TARGETS vst3
        UI_TYPE opengl
        FILES_DSP plugin/BigKnobPlugin.cpp
        FILES_UI  plugin/BigKnobUI.cpp)
    target_compile_definitions(BigKnob PUBLIC DISTRHO_PLUGIN_HAS_UI=1)
else()
    dpf_add_plugin(BigKnob
        TARGETS vst3
        FILES_DSP plugin/BigKnobPlugin.cpp)
    target_compile_definitions(BigKnob PUBLIC DISTRHO_PLUGIN_HAS_UI=0)
endif()

target_include_directories(BigKnob PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/plugin
    ${CMAKE_CURRENT_SOURCE_DIR}/dsp)

# Same targeted fast-math subset as moses-dpf: reassociation/FMA without
# -ffinite-math-only, so the NaN self-heal guards keep working and the
# explicit FTZ in DenormalGuard.hpp still governs denormals.
if(TARGET BigKnob-dsp)
    target_compile_options(BigKnob-dsp PRIVATE -funsafe-math-optimizations)
endif()

if(BIGKNOB_BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
