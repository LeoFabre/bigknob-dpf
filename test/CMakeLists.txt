add_executable(test_bigknob_dsp test_dsp.cpp)
target_include_directories(test_bigknob_dsp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../dsp)
add_test(NAME dsp COMMAND test_bigknob_dsp)
