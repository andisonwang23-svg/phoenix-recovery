// ============================================================================
// PHOENIX RECOVERY — automatic unloaded servo extension/retraction test.
//
// Hardware-free sequencer.  The firmware applies these requests through the
// same calibrated, slew-limited bench-servo path as the proven manual buttons.
// It is never a flight mode and never bypasses the preflight interlock.
// ============================================================================
#pragma once

#include <cstdint>

namespace logic {

enum class ServoRetractTestStatus : uint8_t {
    IDLE = 0,
    SERVO1_EXTEND = 1,
    SERVO1_RETRACT = 2,
    SERVO2_EXTEND = 3,
    SERVO2_RETRACT = 4,
    COMPLETE = 5,
    INTERLOCKED = 6
};

inline const char* servoRetractTestStatusName(ServoRetractTestStatus status) {
    switch (status) {
        case ServoRetractTestStatus::IDLE: return "IDLE";
        case ServoRetractTestStatus::SERVO1_EXTEND: return "SERVO1_EXTEND";
        case ServoRetractTestStatus::SERVO1_RETRACT: return "SERVO1_RETRACT";
        case ServoRetractTestStatus::SERVO2_EXTEND: return "SERVO2_EXTEND";
        case ServoRetractTestStatus::SERVO2_RETRACT: return "SERVO2_RETRACT";
        case ServoRetractTestStatus::COMPLETE: return "COMPLETE";
        case ServoRetractTestStatus::INTERLOCKED: return "INTERLOCKED";
    }
    return "UNKNOWN";
}

struct ServoRetractTestConfig {
    float extension_command = 0.20f;  // unloaded bench fraction
    uint32_t extension_hold_ms = 700;
    uint32_t retraction_settle_ms = 900;
};

struct ServoRetractTestInput {
    uint32_t now_ms = 0;
    bool preflight_allowed = false;
    bool servo_healthy = false;
};

struct ServoRetractTestOutput {
    ServoRetractTestStatus status = ServoRetractTestStatus::IDLE;
    bool active = false;
    bool neutral = true;
    bool completed = false;
    bool interlocked = false;
    float servo1_command = 0.0f;
    float servo2_command = 0.0f;
};

class ServoRetractTest {
public:
    explicit ServoRetractTest(const ServoRetractTestConfig& config = ServoRetractTestConfig());

    ServoRetractTestOutput start(uint32_t now_ms, bool preflight_allowed,
                                 bool servo_healthy);
    ServoRetractTestOutput stop();
    ServoRetractTestOutput step(const ServoRetractTestInput& input);
    const ServoRetractTestOutput& output() const { return output_; }

private:
    ServoRetractTestConfig config_;
    ServoRetractTestOutput output_;
    uint32_t phase_started_ms_ = 0;

    void enter(ServoRetractTestStatus status, uint32_t now_ms);
    static bool elapsed(uint32_t now_ms, uint32_t since_ms, uint32_t duration_ms);
};

} // namespace logic
