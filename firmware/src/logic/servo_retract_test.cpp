#include "servo_retract_test.h"

#include <algorithm>
#include <cmath>

namespace logic {

ServoRetractTest::ServoRetractTest(const ServoRetractTestConfig& config)
    : config_(config) {
    if (!std::isfinite(config_.extension_command)) config_.extension_command = 0.0f;
    config_.extension_command = std::max(0.0f, std::min(config_.extension_command, 0.40f));
}

bool ServoRetractTest::elapsed(uint32_t now_ms, uint32_t since_ms,
                               uint32_t duration_ms) {
    return static_cast<uint32_t>(now_ms - since_ms) >= duration_ms;
}

void ServoRetractTest::enter(ServoRetractTestStatus status, uint32_t now_ms) {
    output_ = ServoRetractTestOutput{};
    output_.status = status;
    phase_started_ms_ = now_ms;

    switch (status) {
        case ServoRetractTestStatus::SERVO1_EXTEND:
            output_.active = true;
            output_.neutral = false;
            output_.servo1_command = config_.extension_command;
            break;
        case ServoRetractTestStatus::SERVO1_RETRACT:
        case ServoRetractTestStatus::SERVO2_RETRACT:
            // Remain actuator owner while the servo controller slews to zero.
            output_.active = true;
            output_.neutral = true;
            break;
        case ServoRetractTestStatus::SERVO2_EXTEND:
            output_.active = true;
            output_.neutral = false;
            output_.servo2_command = config_.extension_command;
            break;
        case ServoRetractTestStatus::COMPLETE:
            output_.completed = true;
            break;
        case ServoRetractTestStatus::INTERLOCKED:
            output_.interlocked = true;
            break;
        case ServoRetractTestStatus::IDLE:
            break;
    }
}

ServoRetractTestOutput ServoRetractTest::start(uint32_t now_ms,
                                               bool preflight_allowed,
                                               bool servo_healthy) {
    if (!preflight_allowed || !servo_healthy) {
        enter(ServoRetractTestStatus::INTERLOCKED, now_ms);
        return output_;
    }
    enter(ServoRetractTestStatus::SERVO1_EXTEND, now_ms);
    return output_;
}

ServoRetractTestOutput ServoRetractTest::stop() {
    enter(ServoRetractTestStatus::IDLE, 0);
    return output_;
}

ServoRetractTestOutput ServoRetractTest::step(const ServoRetractTestInput& input) {
    if (!output_.active) return output_;
    if (!input.preflight_allowed || !input.servo_healthy) {
        enter(ServoRetractTestStatus::INTERLOCKED, input.now_ms);
        return output_;
    }

    switch (output_.status) {
        case ServoRetractTestStatus::SERVO1_EXTEND:
            if (elapsed(input.now_ms, phase_started_ms_, config_.extension_hold_ms))
                enter(ServoRetractTestStatus::SERVO1_RETRACT, input.now_ms);
            break;
        case ServoRetractTestStatus::SERVO1_RETRACT:
            if (elapsed(input.now_ms, phase_started_ms_, config_.retraction_settle_ms))
                enter(ServoRetractTestStatus::SERVO2_EXTEND, input.now_ms);
            break;
        case ServoRetractTestStatus::SERVO2_EXTEND:
            if (elapsed(input.now_ms, phase_started_ms_, config_.extension_hold_ms))
                enter(ServoRetractTestStatus::SERVO2_RETRACT, input.now_ms);
            break;
        case ServoRetractTestStatus::SERVO2_RETRACT:
            if (elapsed(input.now_ms, phase_started_ms_, config_.retraction_settle_ms))
                enter(ServoRetractTestStatus::COMPLETE, input.now_ms);
            break;
        default:
            break;
    }
    return output_;
}

} // namespace logic
