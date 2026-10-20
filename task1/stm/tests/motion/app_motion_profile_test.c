/* Host-buildable regression test for the motion-profiles + Stage SFF1/
 * SFF2/SFF3 work: fresh-boot (AppMotion_Init()) state, PROFILE,PRECISION /
 * PROFILE,STANDARD atomic restore (including both F and B speed-aware
 * servo feed-forward), manual SFF SET marking PROFILE,CUSTOM,
 * AppMotion_ApplyProfile()'s own internal SFF apply NOT marking CUSTOM,
 * and F/B SFF independence -- exercised directly against the real
 * Core/Src/app_motion.c (+ Core/Src/app_pid.c, itself already HAL-free),
 * unlike tests/parser/ which only ever exercises Core/Src/vision_rx.c in
 * isolation.
 *
 * app_motion.c's only non-portable dependency is FreeRTOS.h/task.h (for
 * taskENTER_CRITICAL()/taskEXIT_CRITICAL() and TickType_t/
 * xTaskGetTickCount()/pdMS_TO_TICKS()) -- see tests/motion/fakes/ for
 * minimal host stand-ins. app_motor.h/app_encoder.h/app_servo.h/
 * app_imu.h are the REAL headers from Core/Inc (plain stdint/stdbool, no
 * HAL coupling); tests/motion/fakes/hw_stubs.c provides trivial (never
 * actually exercised by this test's own assertions) definitions for the
 * handful of hardware functions AppMotion_Init()/AppMotion_HandleCommand()
 * reference, since -Wl,--gc-sections on this host's linker does not defer
 * undefined-symbol resolution for sections it later discards the way
 * some ELF linkers do -- confirmed empirically, real stub bodies are
 * required here, not just declarations.
 *
 * Calls the real AppMotion_Init() once, at the very start of main(),
 * before any check -- this is deliberate, not incidental: speedPID's
 * Kp/Ki (unlike every SFF/
 * profile-owned bool-and-three-floats static) has NO static initializer
 * of its own in app_motion.c -- it is only ever established by
 * AppMotion_Init()'s own PID_Init() call, so skipping Init() would leave
 * speedPID.kp/ki at C's implicit zero-init rather than the validated
 * PRECISION values. Calling the real Init() removes that whole class of
 * "is this static initializer really equivalent to a fresh boot"
 * assumption for every field this suite checks, at the one-time cost of
 * the hw_stubs.c stub definitions above.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -Itests/motion/fakes \
 *       -o tests/motion/app_motion_profile_test.exe \
 *       tests/motion/app_motion_profile_test.c Core/Src/app_motion.c \
 *       Core/Src/app_pid.c tests/motion/fakes/hw_stubs.c
 *   ./tests/motion/app_motion_profile_test.exe
 */

#include "app_motion.h"

#include <stdio.h>
#include <string.h>

static unsigned checks;
static unsigned failures;

#define CHECK(expr) do { \
    checks++; \
    if (!(expr)) { \
        failures++; \
        printf("FAIL line %d: %s\n", __LINE__, #expr); \
    } \
} while (0)

#define CHECK_FLOAT_EQ(actual, expected) \
    CHECK(((actual) > ((expected) - 0.001f)) && ((actual) < ((expected) + 0.001f)))

static void expect_sff(
    MotionType direction,
    float expectedStart,
    float expectedFull,
    float expectedMaxUs,
    bool expectedEnable
)
{
    float startSpeed;
    float fullSpeed;
    float maxUs;
    bool  enable;

    AppMotion_GetServoFeedForward(direction, &startSpeed, &fullSpeed, &maxUs, &enable);

    CHECK_FLOAT_EQ(startSpeed, expectedStart);
    CHECK_FLOAT_EQ(fullSpeed, expectedFull);
    CHECK_FLOAT_EQ(maxUs, expectedMaxUs);
    CHECK(enable == expectedEnable);
}

static void expect_precision_values(void)
{
    float targetSpeed, kp, ki, decelStartCounts, approachSpeed;
    float brakeLeadCounts, brakeEntrySpeed;
    bool enable;

    AppMotion_GetSpeedControl(&targetSpeed, &kp, &ki, &enable);
    CHECK_FLOAT_EQ(targetSpeed, 4000.0f);
    CHECK_FLOAT_EQ(kp, 0.02f);
    CHECK_FLOAT_EQ(ki, 0.03f);
    CHECK(enable == true);

    AppMotion_GetSpeedProfile(&decelStartCounts, &approachSpeed, &enable);
    CHECK_FLOAT_EQ(decelStartCounts, 4500.0f);
    CHECK_FLOAT_EQ(approachSpeed, 1200.0f);
    CHECK(enable == true);

    AppMotion_GetStopControl(&brakeLeadCounts, &brakeEntrySpeed, &enable);
    CHECK_FLOAT_EQ(brakeLeadCounts, 150.0f);
    CHECK_FLOAT_EQ(brakeEntrySpeed, 1700.0f);
    CHECK(enable == true);

    expect_sff(MOTION_FORWARD, 3200.0f, 4500.0f, -12.0f, true);
    expect_sff(MOTION_BACKWARD, 3200.0f, 4500.0f, 9.0f, true);

    CHECK(AppMotion_GetProfile() == APP_MOTION_PROFILE_PRECISION);
}

static void expect_standard_values(void)
{
    float targetSpeed, kp, ki, decelStartCounts, approachSpeed;
    float brakeLeadCounts, brakeEntrySpeed;
    bool enable;

    AppMotion_GetSpeedControl(&targetSpeed, &kp, &ki, &enable);
    CHECK_FLOAT_EQ(targetSpeed, 6000.0f);
    CHECK_FLOAT_EQ(kp, 0.02f);
    CHECK_FLOAT_EQ(ki, 0.03f);
    CHECK(enable == true);

    AppMotion_GetSpeedProfile(&decelStartCounts, &approachSpeed, &enable);
    CHECK_FLOAT_EQ(decelStartCounts, 6000.0f);
    CHECK_FLOAT_EQ(approachSpeed, 1200.0f);
    CHECK(enable == true);

    AppMotion_GetStopControl(&brakeLeadCounts, &brakeEntrySpeed, &enable);
    CHECK_FLOAT_EQ(brakeLeadCounts, 150.0f);
    CHECK_FLOAT_EQ(brakeEntrySpeed, 1800.0f);
    CHECK(enable == true);

    expect_sff(MOTION_FORWARD, 3200.0f, 4500.0f, -12.0f, true);
    expect_sff(MOTION_BACKWARD, 3200.0f, 4500.0f, 9.0f, true);

    CHECK(AppMotion_GetProfile() == APP_MOTION_PROFILE_STANDARD);
}

int main(void)
{
    /* --- 1. Fresh boot (the real AppMotion_Init(), see the file header
     * comment) must already match PRECISION exactly, including both F
     * and B SFF. ------------------------------------------------------ */
    AppMotion_Init();
    expect_precision_values();

    /* --- 2. PROFILE,STANDARD must atomically restore ALL Standard
     * parameters, including both F and B SFF, and must not remain
     * (or become) CUSTOM. First perturb state away from any profile so
     * this genuinely proves a reset, not a coincidence. ------------- */
    AppMotion_SetServoFeedForward(MOTION_FORWARD, 1.0f, 2.0f, 55.0f, false);
    AppMotion_SetServoFeedForward(MOTION_BACKWARD, 3.0f, 4.0f, -66.0f, false);
    CHECK(AppMotion_GetProfile() == APP_MOTION_PROFILE_CUSTOM);

    AppMotion_ApplyProfile(APP_MOTION_PROFILE_STANDARD);
    expect_standard_values();

    /* --- 3. PROFILE,PRECISION must atomically restore ALL Precision
     * parameters, including both F and B SFF, and must not remain (or
     * become) CUSTOM. ------------------------------------------------ */
    AppMotion_ApplyProfile(APP_MOTION_PROFILE_PRECISION);
    expect_precision_values();

    /* --- 4. F/B SFF independence: SFF,F touches only Forward; SFF,B
     * touches only Backward; no auto sign inversion (maxUs stored
     * exactly as given, positive or negative, per direction). -------- */
    AppMotion_SetServoFeedForward(MOTION_FORWARD, 1000.0f, 2000.0f, -50.0f, true);
    expect_sff(MOTION_FORWARD, 1000.0f, 2000.0f, -50.0f, true);
    expect_sff(MOTION_BACKWARD, 3200.0f, 4500.0f, 9.0f, true); /* untouched by the F call above */

    AppMotion_SetServoFeedForward(MOTION_BACKWARD, 3000.0f, 4000.0f, 77.0f, false);
    expect_sff(MOTION_BACKWARD, 3000.0f, 4000.0f, 77.0f, false);
    expect_sff(MOTION_FORWARD, 1000.0f, 2000.0f, -50.0f, true); /* untouched by the B call above */

    /* --- 5/6. Manual F SFF and manual B SFF each mark CUSTOM. Restore
     * to PRECISION first so each check starts from a known non-CUSTOM
     * state. ----------------------------------------------------------- */
    AppMotion_ApplyProfile(APP_MOTION_PROFILE_PRECISION);
    CHECK(AppMotion_GetProfile() == APP_MOTION_PROFILE_PRECISION);
    AppMotion_SetServoFeedForward(MOTION_FORWARD, 3200.0f, 4500.0f, -12.0f, true); /* same values, still a manual SET */
    CHECK(AppMotion_GetProfile() == APP_MOTION_PROFILE_CUSTOM);

    AppMotion_ApplyProfile(APP_MOTION_PROFILE_PRECISION);
    CHECK(AppMotion_GetProfile() == APP_MOTION_PROFILE_PRECISION);
    AppMotion_SetServoFeedForward(MOTION_BACKWARD, 3200.0f, 4500.0f, 9.0f, true); /* same values, still a manual SET */
    CHECK(AppMotion_GetProfile() == APP_MOTION_PROFILE_CUSTOM);

    /* --- 7. Applying a named profile after CUSTOM restores that exact
     * profile, not CUSTOM (already exercised by steps 2/3/5/6 above, one
     * more direct assertion here for clarity). ----------------------- */
    AppMotion_ApplyProfile(APP_MOTION_PROFILE_STANDARD);
    CHECK(AppMotion_GetProfile() == APP_MOTION_PROFILE_STANDARD);

    /* --- 8. Invalid direction is rejected (a safe no-op) at the
     * app_motion.c layer itself -- complements (does not duplicate)
     * tests/parser/vision_rx_sff_test.c's own coverage of the wire
     * protocol rejecting a bad 'F'/'B' token before it ever reaches this
     * function. Confirm neither direction's state nor the active
     * profile changes when called with a MotionType that is neither
     * MOTION_FORWARD nor MOTION_BACKWARD. ------------------------------ */
    AppMotion_ApplyProfile(APP_MOTION_PROFILE_PRECISION);
    AppMotion_SetServoFeedForward(MOTION_STOP, 1.0f, 2.0f, 3.0f, true);
    CHECK(AppMotion_GetProfile() == APP_MOTION_PROFILE_PRECISION); /* not marked CUSTOM */
    expect_sff(MOTION_FORWARD, 3200.0f, 4500.0f, -12.0f, true);    /* untouched */
    expect_sff(MOTION_BACKWARD, 3200.0f, 4500.0f, 9.0f, true);     /* untouched */

    /* --- 9. PRESTEER/TURNSPEED/ACKERMANN/TURNBRAKE are turn-only and
     * deliberately NOT profile-owned: PROFILE,PRECISION and
     * PROFILE,STANDARD must never read or write any of them, in either
     * direction. Perturb all four away from both their boot defaults
     * and each other, apply STANDARD then PRECISION, and confirm none
     * of the four moved. -------------------------------------------- */
    {
        float durationMs, k, lookaheadMs;
        bool preSteerEn, ackermannEn, turnBrakeEn;
        uint32_t turnSpeed;

        AppMotion_ApplyProfile(APP_MOTION_PROFILE_PRECISION);
        AppMotion_SetPreSteer(77.0f, false);
        AppMotion_SetTurnSpeed(900U);
        AppMotion_SetAckermann(0.15f, false);
        AppMotion_SetTurnBrake(250.0f, false);

        AppMotion_ApplyProfile(APP_MOTION_PROFILE_STANDARD);
        CHECK(AppMotion_GetProfile() == APP_MOTION_PROFILE_STANDARD);
        AppMotion_GetPreSteer(&durationMs, &preSteerEn);
        CHECK_FLOAT_EQ(durationMs, 77.0f);
        CHECK(preSteerEn == false);
        AppMotion_GetTurnSpeed(&turnSpeed);
        CHECK(turnSpeed == 900U);
        AppMotion_GetAckermann(&k, &ackermannEn);
        CHECK_FLOAT_EQ(k, 0.15f);
        CHECK(ackermannEn == false);
        AppMotion_GetTurnBrake(&lookaheadMs, &turnBrakeEn);
        CHECK_FLOAT_EQ(lookaheadMs, 250.0f);
        CHECK(turnBrakeEn == false);

        AppMotion_ApplyProfile(APP_MOTION_PROFILE_PRECISION);
        CHECK(AppMotion_GetProfile() == APP_MOTION_PROFILE_PRECISION);
        AppMotion_GetPreSteer(&durationMs, &preSteerEn);
        CHECK_FLOAT_EQ(durationMs, 77.0f);
        CHECK(preSteerEn == false);
        AppMotion_GetTurnSpeed(&turnSpeed);
        CHECK(turnSpeed == 900U);
        AppMotion_GetAckermann(&k, &ackermannEn);
        CHECK_FLOAT_EQ(k, 0.15f);
        CHECK(ackermannEn == false);
        AppMotion_GetTurnBrake(&lookaheadMs, &turnBrakeEn);
        CHECK_FLOAT_EQ(lookaheadMs, 250.0f);
        CHECK(turnBrakeEn == false);
    }

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
