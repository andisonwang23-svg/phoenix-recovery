// ============================================================================
// PHOENIX RECOVERY — Filters (pure C++, zero hardware deps).
// Moving average, low-pass, complementary estimator.
// All functions are host-testable.
// ============================================================================
#pragma once

#include <cstdint>
#include <array>

namespace logic {

// ---------------------------------------------------------------------------
// Simple Moving Average (SMA) filter
// ---------------------------------------------------------------------------
template <typename T, size_t N>
class MovingAverage {
    static_assert(N > 0, "Window size must be > 0");

    std::array<T, N> samples_{};
    size_t index_ = 0;
    size_t count_ = 0;
    T sum_ = T(0);

public:
    MovingAverage() = default;

    void reset() {
        sum_ = T(0);
        index_ = 0;
        count_ = 0;
    }

    void add(T value) {
        sum_ -= samples_[index_];
        samples_[index_] = value;
        sum_ += value;
        index_ = (index_ + 1) % N;
        if (count_ < N) ++count_;
    }

    T value() const {
        return count_ > 0 ? sum_ / static_cast<T>(count_) : T(0);
    }

    size_t count() const { return count_; }
    bool full() const { return count_ == N; }
};

// ---------------------------------------------------------------------------
// First-order low-pass filter (exponential smoothing)
// y[n] = α * x[n] + (1-α) * y[n-1]
// ---------------------------------------------------------------------------
template <typename T = float>
class LowPassFilter {
    T y_ = T(0);
    T alpha_;
    bool initialized_ = false;

public:
    explicit LowPassFilter(T alpha = T(0.1)) : alpha_(alpha) {}
    void setAlpha(T alpha) { alpha_ = alpha; }

    void reset(T initial = T(0)) {
        y_ = initial;
        initialized_ = true;
    }

    T update(T x) {
        if (!initialized_) {
            y_ = x;
            initialized_ = true;
        } else {
            y_ = alpha_ * x + (T(1) - alpha_) * y_;
        }
        return y_;
    }

    T value() const { return y_; }
    bool initialized() const { return initialized_; }
};

// ---------------------------------------------------------------------------
// Complementary filter for vertical velocity estimation
// Combines barometric altitude (low freq) with accelerometer (high freq)
// ---------------------------------------------------------------------------
class VerticalSpeedEstimator {
    float vs_ = 0.0f;       // estimated vertical speed (m/s, +up)
    float alt_ = 0.0f;      // filtered altitude (m)
    float gain_alt_;        // gain for altitude correction
    float gain_vs_;         // gain for vs correction
    bool initialized_ = false;

public:
    VerticalSpeedEstimator(float gain_alt = 0.20f, float gain_vs = 0.10f)
        : gain_alt_(gain_alt), gain_vs_(gain_vs) {}

    void setGains(float gain_alt, float gain_vs) {
        gain_alt_ = gain_alt;
        gain_vs_ = gain_vs;
    }

    void reset(float initial_alt = 0.0f, float initial_vs = 0.0f) {
        alt_ = initial_alt;
        vs_ = initial_vs;
        initialized_ = true;
    }

    // Called at each estimator tick with filtered altitude
    float update(float filtered_altitude, float dt_s) {
        if (!initialized_) {
            alt_ = filtered_altitude;
            vs_ = 0.0f;
            initialized_ = true;
            return 0.0f;
        }

        // Predict step
        float alt_pred = alt_ + vs_ * dt_s;

        // Correct using measured filtered altitude
        float alt_error = filtered_altitude - alt_pred;
        alt_ = alt_pred + gain_alt_ * alt_error;
        vs_ = vs_ + gain_vs_ * alt_error / dt_s;

        return vs_;
    }

    float verticalSpeed() const { return vs_; }
    float altitude() const { return alt_; }
    bool initialized() const { return initialized_; }
};

// ---------------------------------------------------------------------------
// Rate limiter for command signals
// Limits |Δcommand| per second
// ---------------------------------------------------------------------------
class RateLimiter {
    float last_value_ = 0.0f;
    float max_rate_per_s_;
    bool initialized_ = false;

public:
    explicit RateLimiter(float max_rate_per_s = 1.0f)
        : max_rate_per_s_(max_rate_per_s) {}

    void setMaxRate(float rate) { max_rate_per_s_ = rate; }

    void reset(float initial = 0.0f) {
        last_value_ = initial;
        initialized_ = true;
    }

    float update(float desired, float dt_s) {
        if (!initialized_) {
            last_value_ = desired;
            initialized_ = true;
            return desired;
        }

        float max_delta = max_rate_per_s_ * dt_s;
        float delta = desired - last_value_;

        if (delta > max_delta) delta = max_delta;
        else if (delta < -max_delta) delta = -max_delta;

        last_value_ += delta;
        return last_value_;
    }

    float value() const { return last_value_; }
    bool initialized() const { return initialized_; }
};

// ---------------------------------------------------------------------------
// Deadband + clamp (for steering commands)
// ---------------------------------------------------------------------------
inline float applyDeadbandAndClamp(float input, float deadband, float max_mag) {
    if (input > -deadband && input < deadband) return 0.0f;
    if (input > max_mag) return max_mag;
    if (input < -max_mag) return -max_mag;
    return input;
}

// ---------------------------------------------------------------------------
// Reversal guard - prevents rapid direction changes
// Returns true if reversal is allowed
// ---------------------------------------------------------------------------
class ReversalGuard {
    int last_sign_ = 0;  // -1, 0, +1
    uint32_t last_change_ms_ = 0;
    uint32_t guard_ms_;

public:
    explicit ReversalGuard(uint32_t guard_ms = 800) : guard_ms_(guard_ms) {}

    void setGuardTime(uint32_t ms) { guard_ms_ = ms; }

    void reset() {
        last_sign_ = 0;
        last_change_ms_ = 0;
    }

    bool allow(float value, uint32_t now_ms) {
        int sign = (value > 0) ? 1 : (value < 0 ? -1 : 0);

        if (sign == 0) {
            last_sign_ = 0;
            return true;
        }

        if (sign != last_sign_ && last_sign_ != 0) {
            if (now_ms - last_change_ms_ < guard_ms_) {
                return false;  // reversal blocked
            }
        }

        if (sign != last_sign_) {
            last_change_ms_ = now_ms;
            last_sign_ = sign;
        }
        return true;
    }
};

} // namespace logic