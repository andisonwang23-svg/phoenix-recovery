#include <unity.h>

#include "logic/lora_airtime.h"

using namespace logic;

void v8_size_frame_has_expected_sf7_airtime() {
    // Packed telemetry v8 is 140 bytes. SF7/BW125/CR4/5 remains below the
    // 250 ms airtime budget needed by the 2 Hz half-duplex command schedule.
    const uint32_t airtime_us = loraPacketAirtimeUs(140, 7, 125000, 5);
    TEST_ASSERT_UINT32_WITHIN(1000, 230656, airtime_us);
}

void five_hz_starves_command_receiver() {
    TEST_ASSERT_FALSE(loraScheduleLeavesReceiveWindow(
        140, 7, 125000, 5, 5, 250));
}

void two_hz_reserves_command_receiver() {
    TEST_ASSERT_TRUE(loraScheduleLeavesReceiveWindow(
        140, 7, 125000, 5, 2, 250));
}

void invalid_radio_parameters_fail_closed() {
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX,
        loraPacketAirtimeUs(140, 7, 0, 5));
    TEST_ASSERT_FALSE(loraScheduleLeavesReceiveWindow(
        140, 7, 0, 5, 2, 250));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(v8_size_frame_has_expected_sf7_airtime);
    RUN_TEST(five_hz_starves_command_receiver);
    RUN_TEST(two_hz_reserves_command_receiver);
    RUN_TEST(invalid_radio_parameters_fail_closed);
    return UNITY_END();
}
