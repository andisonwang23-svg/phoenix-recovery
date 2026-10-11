#include <unity.h>

#include "logic/guided_drop_arm_gate.h"
#include "../../firmware/src/logic/guided_drop_arm_gate.cpp"

using namespace logic;

static GuidedDropUnlocks unlocked20() {
    GuidedDropUnlocks u;
    u.feature_enabled = true;
    u.calibration_confirmed = true;
    u.twenty_percent_unlocked = true;
    return u;
}

static GuidedArmPrerequisites ready() {
    GuidedArmPrerequisites p;
    p.physical_arm_active = true;
    p.target_valid = true;
    p.gps_valid = true;
    p.imu_valid = true;
    p.barometer_valid = true;
    p.servo_valid = true;
    p.log_ready = true;
    p.recorder_idle = true;
    return p;
}

void requires_canopy_and_physical_arm() {
    GuidedDropArmGate gate(5000);
    auto out = gate.prepare(100, GuidedAuthorityLevel::TWENTY_PERCENT,
                            unlocked20(), ready());
    TEST_ASSERT_EQUAL((int)GuidedArmStatus::REJECTED_CANOPY, (int)out.status);
    gate.confirmCanopy(120, true);
    auto prerequisites = ready();
    prerequisites.physical_arm_active = false;
    out = gate.prepare(200, GuidedAuthorityLevel::TWENTY_PERCENT,
                       unlocked20(), prerequisites);
    TEST_ASSERT_EQUAL((int)GuidedArmStatus::REJECTED_PHYSICAL_ARM, (int)out.status);
}

void authority_progression_is_fail_closed() {
    GuidedDropArmGate gate;
    gate.confirmCanopy(100, true);
    auto out = gate.prepare(200, GuidedAuthorityLevel::FORTY_PERCENT,
                            unlocked20(), ready());
    TEST_ASSERT_EQUAL((int)GuidedArmStatus::REJECTED_AUTHORITY_LOCK, (int)out.status);
    out = gate.prepare(300, GuidedAuthorityLevel::VALIDATED_FULL,
                       unlocked20(), ready());
    TEST_ASSERT_EQUAL((int)GuidedArmStatus::REJECTED_AUTHORITY_LOCK, (int)out.status);
}

void two_step_confirmation_arms_inside_window() {
    GuidedDropArmGate gate(5000);
    gate.confirmCanopy(100, true);
    auto out = gate.prepare(200, GuidedAuthorityLevel::TWENTY_PERCENT,
                            unlocked20(), ready());
    TEST_ASSERT_TRUE(out.confirmation_pending);
    out = gate.confirmAndArm(5100, unlocked20(), ready());
    TEST_ASSERT_TRUE(out.arm_authorized);
    TEST_ASSERT_EQUAL((int)GuidedArmStatus::ARMED, (int)out.status);
}

void expired_or_changed_prerequisites_cannot_arm() {
    GuidedDropArmGate gate(5000);
    gate.confirmCanopy(100, true);
    gate.prepare(200, GuidedAuthorityLevel::TWENTY_PERCENT, unlocked20(), ready());
    auto out = gate.confirmAndArm(5201, unlocked20(), ready());
    TEST_ASSERT_FALSE(out.arm_authorized);
    TEST_ASSERT_EQUAL((int)GuidedArmStatus::REJECTED_CONFIRMATION, (int)out.status);

    gate.prepare(6000, GuidedAuthorityLevel::TWENTY_PERCENT, unlocked20(), ready());
    auto prerequisites = ready();
    prerequisites.gps_valid = false;
    out = gate.confirmAndArm(6100, unlocked20(), prerequisites);
    TEST_ASSERT_FALSE(out.arm_authorized);
    TEST_ASSERT_EQUAL((int)GuidedArmStatus::REJECTED_TARGET, (int)out.status);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(requires_canopy_and_physical_arm);
    RUN_TEST(authority_progression_is_fail_closed);
    RUN_TEST(two_step_confirmation_arms_inside_window);
    RUN_TEST(expired_or_changed_prerequisites_cannot_arm);
    return UNITY_END();
}
