// ============================================================================
// PHOENIX RECOVERY — Automatic ground-station drop-log session tracker.
//
// Hardware-free so the start/append/close policy can be tested on the host.
// This tracker never affects flight control; it only decides how received
// telemetry should be mirrored into the ground station's persistent log.
// ============================================================================
#pragma once

#include <cstdint>

#include "logic/drop_test_controller.h"

namespace logic {

enum class DropLogCloseReason : uint8_t {
    NONE = 0,
    COMPLETE,
    ABORTED,
    INTERRUPTED_BY_NEW_TEST
};

struct DropLogObservation {
    uint16_t test_id = 0;
    DropTestState state = DropTestState::IDLE;
    bool recording = false;
    uint32_t payload_time_ms = 0;
    uint8_t packet_sequence = 0;
};

struct DropLogDecision {
    bool close_previous = false;
    bool start_new = false;
    bool append_sample = false;
    bool close_after_sample = false;
    uint16_t test_id = 0;
    DropLogCloseReason close_reason = DropLogCloseReason::NONE;
};

class DropLogSession {
public:
    DropLogDecision observe(const DropLogObservation& observation) {
        DropLogDecision decision;
        const bool terminal = isTerminal(observation.state);
        const bool identified = observation.test_id != 0;

        if (active_ && identified && observation.test_id != active_test_id_) {
            if (observation.recording || terminal) {
                decision.close_previous = true;
                decision.close_reason = DropLogCloseReason::INTERRUPTED_BY_NEW_TEST;
                active_ = false;
                have_last_sample_ = false;
            }
        }

        if (!active_) {
            // A terminal sample may be the only packet received after a short
            // radio outage. Preserve it as a one-row recovered log instead of
            // silently losing all evidence of that test.
            const bool unseen_terminal = terminal && identified &&
                (observation.test_id != last_terminal_test_id_ ||
                 observation.payload_time_ms != last_terminal_payload_time_ms_ ||
                 observation.packet_sequence != last_terminal_packet_sequence_);
            if (identified && (observation.recording || unseen_terminal)) {
                active_ = true;
                active_test_id_ = observation.test_id;
                have_last_sample_ = false;
                decision.start_new = true;
            }
        }

        if (!active_ || observation.test_id != active_test_id_) {
            decision.test_id = active_test_id_;
            return decision;
        }

        decision.test_id = active_test_id_;
        const bool duplicate = have_last_sample_ &&
            last_payload_time_ms_ == observation.payload_time_ms &&
            last_packet_sequence_ == observation.packet_sequence;
        if (!duplicate) {
            decision.append_sample = true;
            have_last_sample_ = true;
            last_payload_time_ms_ = observation.payload_time_ms;
            last_packet_sequence_ = observation.packet_sequence;
        }

        if (terminal) {
            decision.close_after_sample = true;
            decision.close_reason = observation.state == DropTestState::TEST_ABORTED
                ? DropLogCloseReason::ABORTED
                : DropLogCloseReason::COMPLETE;
            last_terminal_test_id_ = active_test_id_;
            last_terminal_payload_time_ms_ = observation.payload_time_ms;
            last_terminal_packet_sequence_ = observation.packet_sequence;
            active_ = false;
            have_last_sample_ = false;
        }
        return decision;
    }

    bool active() const { return active_; }
    uint16_t activeTestId() const { return active_test_id_; }

private:
    static bool isTerminal(DropTestState state) {
        return state == DropTestState::TEST_COMPLETE ||
               state == DropTestState::TEST_ABORTED;
    }

    bool active_ = false;
    bool have_last_sample_ = false;
    uint16_t active_test_id_ = 0;
    uint16_t last_terminal_test_id_ = 0;
    uint32_t last_terminal_payload_time_ms_ = 0;
    uint8_t last_terminal_packet_sequence_ = 0;
    uint32_t last_payload_time_ms_ = 0;
    uint8_t last_packet_sequence_ = 0;
};

inline const char* dropLogCloseReasonName(DropLogCloseReason reason) {
    switch (reason) {
        case DropLogCloseReason::NONE: return "NONE";
        case DropLogCloseReason::COMPLETE: return "COMPLETE";
        case DropLogCloseReason::ABORTED: return "ABORTED";
        case DropLogCloseReason::INTERRUPTED_BY_NEW_TEST: return "INTERRUPTED_BY_NEW_TEST";
    }
    return "UNKNOWN";
}

} // namespace logic
