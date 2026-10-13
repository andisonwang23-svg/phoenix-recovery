#include <unity.h>

#include "logic/drop_log_session.h"

using namespace logic;

static DropLogObservation observation(uint16_t id,
                                      DropTestState state,
                                      bool recording,
                                      uint32_t payload_ms,
                                      uint8_t sequence) {
    DropLogObservation value;
    value.test_id = id;
    value.state = state;
    value.recording = recording;
    value.payload_time_ms = payload_ms;
    value.packet_sequence = sequence;
    return value;
}

void idle_telemetry_does_not_create_a_log() {
    DropLogSession session;
    const auto decision = session.observe(
        observation(0, DropTestState::IDLE, false, 100, 1));
    TEST_ASSERT_FALSE(decision.start_new);
    TEST_ASSERT_FALSE(decision.append_sample);
    TEST_ASSERT_FALSE(session.active());
}

void active_drop_starts_and_appends_automatically() {
    DropLogSession session;
    const auto first = session.observe(
        observation(42, DropTestState::ARMED_RECORDING, true, 1000, 7));
    TEST_ASSERT_TRUE(first.start_new);
    TEST_ASSERT_TRUE(first.append_sample);
    TEST_ASSERT_EQUAL_UINT16(42, first.test_id);
    TEST_ASSERT_TRUE(session.active());

    const auto next = session.observe(
        observation(42, DropTestState::DROP_CONFIRMED, true, 1500, 8));
    TEST_ASSERT_FALSE(next.start_new);
    TEST_ASSERT_TRUE(next.append_sample);
    TEST_ASSERT_FALSE(next.close_after_sample);
}

void duplicate_packets_are_not_written_twice() {
    DropLogSession session;
    session.observe(observation(8, DropTestState::ARMED_RECORDING, true, 1000, 4));
    const auto duplicate = session.observe(
        observation(8, DropTestState::ARMED_RECORDING, true, 1000, 4));
    TEST_ASSERT_FALSE(duplicate.append_sample);
    TEST_ASSERT_TRUE(session.active());
}

void complete_and_abort_close_the_matching_log() {
    DropLogSession complete_session;
    complete_session.observe(
        observation(10, DropTestState::ARMED_RECORDING, true, 100, 1));
    const auto complete = complete_session.observe(
        observation(10, DropTestState::TEST_COMPLETE, false, 9000, 2));
    TEST_ASSERT_TRUE(complete.append_sample);
    TEST_ASSERT_TRUE(complete.close_after_sample);
    TEST_ASSERT_EQUAL(static_cast<int>(DropLogCloseReason::COMPLETE),
                      static_cast<int>(complete.close_reason));
    TEST_ASSERT_FALSE(complete_session.active());

    DropLogSession abort_session;
    abort_session.observe(
        observation(11, DropTestState::ARMED_RECORDING, true, 100, 1));
    const auto aborted = abort_session.observe(
        observation(11, DropTestState::TEST_ABORTED, false, 200, 2));
    TEST_ASSERT_TRUE(aborted.close_after_sample);
    TEST_ASSERT_EQUAL(static_cast<int>(DropLogCloseReason::ABORTED),
                      static_cast<int>(aborted.close_reason));
    TEST_ASSERT_FALSE(abort_session.active());
}

void a_new_test_closes_an_interrupted_session_before_starting() {
    DropLogSession session;
    session.observe(observation(21, DropTestState::DROP_CONFIRMED, true, 1000, 1));
    const auto next = session.observe(
        observation(22, DropTestState::ARMED_RECORDING, true, 2000, 2));
    TEST_ASSERT_TRUE(next.close_previous);
    TEST_ASSERT_TRUE(next.start_new);
    TEST_ASSERT_TRUE(next.append_sample);
    TEST_ASSERT_EQUAL(static_cast<int>(DropLogCloseReason::INTERRUPTED_BY_NEW_TEST),
                      static_cast<int>(next.close_reason));
    TEST_ASSERT_EQUAL_UINT16(22, next.test_id);
    TEST_ASSERT_TRUE(session.active());
}

void a_missed_test_still_preserves_its_terminal_packet_once() {
    DropLogSession session;
    const auto recovered = session.observe(
        observation(31, DropTestState::TEST_COMPLETE, false, 5000, 9));
    TEST_ASSERT_TRUE(recovered.start_new);
    TEST_ASSERT_TRUE(recovered.append_sample);
    TEST_ASSERT_TRUE(recovered.close_after_sample);

    const auto repeated = session.observe(
        observation(31, DropTestState::TEST_COMPLETE, false, 5000, 9));
    TEST_ASSERT_FALSE(repeated.start_new);
    TEST_ASSERT_FALSE(repeated.append_sample);

    // Payload test IDs restart after a payload reboot. A later terminal packet
    // with the same ID but a new timestamp/sequence must still be retained.
    const auto reused_id = session.observe(
        observation(31, DropTestState::TEST_ABORTED, false, 800, 1));
    TEST_ASSERT_TRUE(reused_id.start_new);
    TEST_ASSERT_TRUE(reused_id.append_sample);
    TEST_ASSERT_TRUE(reused_id.close_after_sample);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(idle_telemetry_does_not_create_a_log);
    RUN_TEST(active_drop_starts_and_appends_automatically);
    RUN_TEST(duplicate_packets_are_not_written_twice);
    RUN_TEST(complete_and_abort_close_the_matching_log);
    RUN_TEST(a_new_test_closes_an_interrupted_session_before_starting);
    RUN_TEST(a_missed_test_still_preserves_its_terminal_packet_once);
    return UNITY_END();
}
