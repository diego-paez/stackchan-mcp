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
    ../main/boards/stackchan/attention/face_mimic.cc
    ../main/boards/stackchan/attention/idle_scan.cc
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
    ../main/boards/stackchan/attention/face_mimic.cc
    ../main/boards/stackchan/attention/idle_scan.cc
)
target_include_directories(sim_greeting PRIVATE ../main/boards/stackchan)

# --- face mimic ---------------------------------------------------------
# What the screen does about the face the head is following. Separate target
# from attention_safety_test because it answers a separate question: that one
# is about what can reach a servo, this one is about what can reach the
# display. See main/boards/stackchan/attention/face_mimic.h.
add_executable(face_mimic_test
    test_face_mimic.cc
    ../main/boards/stackchan/attention/face_mimic.cc
    ../main/boards/stackchan/attention/idle_scan.cc
    ../main/boards/stackchan/attention/servo_safety_controller.cc
    ../main/boards/stackchan/attention/attention_controller.cc
    ../main/boards/stackchan/attention/motion_controller.cc
    ../main/boards/stackchan/attention/behavior_manager.cc
    ../main/boards/stackchan/attention/motion_mixer.cc
    ../main/boards/stackchan/attention/head_controller.cc
    ../main/boards/stackchan/attention/greeting_routine.cc
)

target_include_directories(face_mimic_test
    PRIVATE ../main/boards/stackchan
)

target_link_libraries(face_mimic_test PRIVATE GTest::gtest_main)

gtest_discover_tests(face_mimic_test)

# Not a test — a simulation you read, for the half of the mimic design that
# assertions cannot reach: whether the response table reads as empathy.
add_executable(sim_mimic
    sim_mimic.cc
    ../main/boards/stackchan/attention/face_mimic.cc
    ../main/boards/stackchan/attention/idle_scan.cc
    ../main/boards/stackchan/attention/servo_safety_controller.cc
    ../main/boards/stackchan/attention/attention_controller.cc
    ../main/boards/stackchan/attention/motion_controller.cc
    ../main/boards/stackchan/attention/behavior_manager.cc
    ../main/boards/stackchan/attention/motion_mixer.cc
    ../main/boards/stackchan/attention/head_controller.cc
    ../main/boards/stackchan/attention/greeting_routine.cc
)
target_include_directories(sim_mimic PRIVATE ../main/boards/stackchan)

# --- idle search cycle ---------------------------------------------------
# Looking around, reacting to a voice, settling on a face. See
# main/boards/stackchan/attention/idle_scan.h.
add_executable(idle_scan_test
    test_idle_scan.cc
    ../main/boards/stackchan/attention/idle_scan.cc
    ../main/boards/stackchan/attention/face_mimic.cc
    ../main/boards/stackchan/attention/idle_scan.cc
    ../main/boards/stackchan/attention/servo_safety_controller.cc
    ../main/boards/stackchan/attention/attention_controller.cc
    ../main/boards/stackchan/attention/motion_controller.cc
    ../main/boards/stackchan/attention/behavior_manager.cc
    ../main/boards/stackchan/attention/motion_mixer.cc
    ../main/boards/stackchan/attention/head_controller.cc
    ../main/boards/stackchan/attention/greeting_routine.cc
)
target_include_directories(idle_scan_test PRIVATE ../main/boards/stackchan)
target_link_libraries(idle_scan_test PRIVATE GTest::gtest_main)
gtest_discover_tests(idle_scan_test)

# --- the drawn face -------------------------------------------------------
# Geometry only: where the eyes and mouth go on a face that fills the panel.
# Header-only, so there is no source to add beyond the test itself. See
# main/boards/stackchan/avatar/face_layout.h.
add_executable(face_layout_test
    test_face_layout.cc
)
target_include_directories(face_layout_test PRIVATE ../main/boards/stackchan)
target_link_libraries(face_layout_test PRIVATE GTest::gtest_main)
gtest_discover_tests(face_layout_test)

# Not a test — a picture you look at. Renders the same geometry and the same
# mouth mask the panel draws, so the face can be judged without a robot.
add_executable(sim_face sim_face.cc)
target_include_directories(sim_face PRIVATE ../main/boards/stackchan)
