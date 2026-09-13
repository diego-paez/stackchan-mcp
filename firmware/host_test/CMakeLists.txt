cmake_minimum_required(VERSION 3.16)

project(stackchan_firmware_host_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.17.0
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(mdns_candidate_extract_test
    test_mdns_candidate_extract.cc
    ../main/protocols/mdns_candidate_extract.cc
)

target_compile_definitions(mdns_candidate_extract_test
    PRIVATE STACKCHAN_MDNS_HOST_TEST
)

target_include_directories(mdns_candidate_extract_test
    PRIVATE
        stubs
        ../main/protocols
)

target_link_libraries(mdns_candidate_extract_test
    PRIVATE GTest::gtest_main
)

add_executable(listening_profile_test
    test_listening_profile.cc
    ../main/listening_profile.cc
)

target_include_directories(listening_profile_test
    PRIVATE
        ../main
)

target_link_libraries(listening_profile_test
    PRIVATE GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(mdns_candidate_extract_test)
gtest_discover_tests(listening_profile_test)

# --- attention / servo-safety -------------------------------------------
# The controllers are deliberately free of ESP-IDF so the safety layer can be
# tested where a mistake costs nothing. See
# main/boards/stackchan/attention/README.md.
add_executable(attention_safety_test
    test_attention_safety.cc
    ../main/boards/stackchan/attention/servo_safety_controller.cc
    ../main/boards/stackchan/attention/attention_controller.cc
    ../main/boards/stackchan/attention/motion_controller.cc
    ../main/boards/stackchan/attention/behavior_manager.cc
    ../main/boards/stackchan/attention/motion_mixer.cc
    ../main/boards/stackchan/attention/head_controller.cc
    ../main/boards/stackchan/attention/greeting_routine.cc
)

target_include_directories(attention_safety_test
    PRIVATE ../main/boards/stackchan
)

target_link_libraries(attention_safety_test PRIVATE GTest::gtest_main)

add_test(NAME attention_safety_test COMMAND attention_safety_test)

# Not a test — a simulation you read. Prints the timeline of a boot: the
# safety self-test, the greeting, then the head following a child who moves
# around and leaves the frame. Nothing is asserted; the tests do that.
add_executable(sim_greeting
    sim_greeting.cc
    ../main/boards/stackchan/attention/servo_safety_controller.cc
    ../main/boards/stackchan/attention/attention_controller.cc
    ../main/boards/stackchan/attention/motion_controller.cc
    ../main/boards/stackchan/attention/behavior_manager.cc
    ../main/boards/stackchan/attention/motion_mixer.cc
    ../main/boards/stackchan/attention/head_controller.cc
    ../main/boards/stackchan/attention/greeting_routine.cc
)
target_include_directories(sim_greeting PRIVATE ../main/boards/stackchan)
