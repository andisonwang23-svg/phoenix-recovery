#include <unity.h>

#include "logic/servo_retract_test.h"
#include "../../firmware/src/logic/servo_retract_test.cpp"

using namespace logic;

static ServoRetractTestConfig config() {
    ServoRetractTestConfig c;
    c.extension_command = 0.20f;
    c.extension_hold_ms = 100;
    c.retraction_settle_ms = 150;
    return c;
}

static ServoRetractTestInput allowed(uint32_t now_ms) {
    ServoRetractTestInput in;
    in.now_ms = now_ms;
    in.preflight_allowed = true;
    in.servo_healthy = true;
    return in;
}

void runs_both_servos_and_retracts_between_extensions() {
    ServoRetractTest test(config());
    auto out = test.start(10, true, true);
    TEST_ASSERT_EQUAL((int)ServoRetractTestStatus::SERVO1_EXTEND, (int)out.status);
    TEST_ASSERT_FLOAT_WITHIN(.001f, .20f, out.servo1_command);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0.0f, out.servo2_command);

    out = test.step(allowed(110));
    TEST_ASSERT_EQUAL((int)ServoRetractTestStatus::SERVO1_RETRACT, (int)out.status);
    TEST_ASSERT_TRUE(out.active);
    TEST_ASSERT_TRUE(out.neutral);

    out = test.step(allowed(260));
    TEST_ASSERT_EQUAL((int)ServoRetractTestStatus::SERVO2_EXTEND, (int)out.status);
    TEST_ASSERT_FLOAT_WITHIN(.001f, .20f, out.servo2_command);

    out = test.step(allowed(360));
    TEST_ASSERT_EQUAL((int)ServoRetractTestStatus::SERVO2_RETRACT, (int)out.status);
    TEST_ASSERT_TRUE(out.neutral);
    out = test.step(allowed(510));
    TEST_ASSERT_EQUAL((int)ServoRetractTestStatus::COMPLETE, (int)out.status);
    TEST_ASSERT_TRUE(out.completed);
    TEST_ASSERT_FALSE(out.active);
    TEST_ASSERT_TRUE(out.neutral);
}

void start_rejects_an_unsafe_preflight_or_bad_servo() {
    ServoRetractTest test(config());
    auto out = test.start(10, false, true);
    TEST_ASSERT_TRUE(out.interlocked);
    TEST_ASSERT_TRUE(out.neutral);
    out = test.start(20, true, false);
    TEST_ASSERT_TRUE(out.interlocked);
    TEST_ASSERT_TRUE(out.neutral);
}

void interlock_during_motion_stops_at_neutral() {
    ServoRetractTest test(config());
    test.start(10, true, true);
    auto in = allowed(20);
    in.preflight_allowed = false;
    auto out = test.step(in);
    TEST_ASSERT_EQUAL((int)ServoRetractTestStatus::INTERLOCKED, (int)out.status);
    TEST_ASSERT_FALSE(out.active);
    TEST_ASSERT_TRUE(out.neutral);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0.0f, out.servo1_command);
    TEST_ASSERT_FLOAT_WITHIN(.001f, 0.0f, out.servo2_command);
}

void stop_is_repeatable_and_neutral() {
    ServoRetractTest test(config());
    test.start(10, true, true);
    auto out = test.stop();
    TEST_ASSERT_EQUAL((int)ServoRetractTestStatus::IDLE, (int)out.status);
    TEST_ASSERT_TRUE(out.neutral);
    out = test.start(20, true, true);
    TEST_ASSERT_TRUE(out.active);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(runs_both_servos_and_retracts_between_extensions);
    RUN_TEST(start_rejects_an_unsafe_preflight_or_bad_servo);
    RUN_TEST(interlock_during_motion_stops_at_neutral);
    RUN_TEST(stop_is_repeatable_and_neutral);
    return UNITY_END();
}
