#include <unity.h>

#include "logic/lora_airtime.h"

using namespace logic;

void v7_size_frame_has_expected_sf7_airtime() {
    // Current packed telemetry v7 is 124 bytes. SF7/BW125/CR4/5 takes about
    // 205 ms including its eight-symbol preamble.
    const uint32_t airtime_us = loraPacketAirtimeUs(124, 7, 125000, 5);
    TEST_ASSERT_UINT32_WITHIN(1000, 205056, airtime_us);
}

void five_hz_starves_command_receiver() {
    TEST_ASSERT_FALSE(loraScheduleLeavesReceiveWindow(
        124, 7, 125000, 5, 5, 250));
}

void two_hz_reserves_command_receiver() {
    TEST_ASSERT_TRUE(loraScheduleLeavesReceiveWindow(
        124, 7, 125000, 5, 2, 250));
}

void invalid_radio_parameters_fail_closed() {
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX,
        loraPacketAirtimeUs(124, 7, 0, 5));
    TEST_ASSERT_FALSE(loraScheduleLeavesReceiveWindow(
        124, 7, 0, 5, 2, 250));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(v7_size_frame_has_expected_sf7_airtime);
    RUN_TEST(five_hz_starves_command_receiver);
    RUN_TEST(two_hz_reserves_command_receiver);
    RUN_TEST(invalid_radio_parameters_fail_closed);
    return UNITY_END();
}
