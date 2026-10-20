/* Host-buildable regression test for the physically validated turn
 * configuration boot defaults: PRESTEER=150.0ms/enabled,
 * TURNSPEED=1700 counts/sec, ACKERMANN=0.270/enabled,
 * TURNBRAKE=135.0ms/enabled. These four groups remain turn-only and
 * deliberately NOT profile-owned (see app_motion_profile_test.c for
 * proof PROFILE,PRECISION/PROFILE,STANDARD never touch them) -- this
 * file instead proves AppMotion_Init() itself now explicitly restores
 * all four to their DEFAULT_* constants (app_motion.c), including a
 * repeated call after runtime changes, so a re-init is deterministic
 * rather than relying solely on each static's one-time file-scope
 * initializer.
 *
 * Exercised directly against the real Core/Src/app_motion.c
 * (+ app_pid.c), the same way the existing tests/motion test files do.
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -Itests/motion/fakes \
 *       -o tests/motion/app_motion_turn_defaults_test.exe \
 *       tests/motion/app_motion_turn_defaults_test.c Core/Src/app_motion.c \
 *       Core/Src/app_pid.c tests/motion/fakes/hw_stubs.c
 *   ./tests/motion/app_motion_turn_defaults_test.exe
 */

#include "app_motion.h"

#include <stdio.h>

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

static void expect_turn_defaults(void)
{
    float durationMs;
    bool preSteerEnabled;
    uint32_t turnSpeed;
    float k;
    bool ackermannEnabled;
    float lookaheadMs;
    bool turnBrakeEnabled;

    AppMotion_GetPreSteer(&durationMs, &preSteerEnabled);
    CHECK_FLOAT_EQ(durationMs, 150.0f);
    CHECK(preSteerEnabled == true);

    AppMotion_GetTurnSpeed(&turnSpeed);
    CHECK(turnSpeed == 1700U);

    AppMotion_GetAckermann(&k, &ackermannEnabled);
    CHECK_FLOAT_EQ(k, 0.270f);
    CHECK(ackermannEnabled == true);

    AppMotion_GetTurnBrake(&lookaheadMs, &turnBrakeEnabled);
    CHECK_FLOAT_EQ(lookaheadMs, 135.0f);
    CHECK(turnBrakeEnabled == true);
}

int main(void)
{
    /* --- 1. Fresh AppMotion_Init() returns the exact physically
     * validated turn defaults through every getter. -------------------- */
    AppMotion_Init();
    expect_turn_defaults();

    /* --- 2. Changing all four settings away from both their defaults
     * and each other, then calling AppMotion_Init() again, must
     * restore the exact same defaults -- proves the restore happens in
     * AppMotion_Init() itself, not merely a one-time static
     * initializer that a second Init() call could never re-establish. */
    AppMotion_SetPreSteer(0.0f, false);
    AppMotion_SetTurnSpeed(2200U);
    AppMotion_SetAckermann(0.45f, false);
    AppMotion_SetTurnBrake(10.0f, false);

    {
        float durationMs;
        bool preSteerEnabled;
        uint32_t turnSpeed;
        float k;
        bool ackermannEnabled;
        float lookaheadMs;
        bool turnBrakeEnabled;

        AppMotion_GetPreSteer(&durationMs, &preSteerEnabled);
        CHECK_FLOAT_EQ(durationMs, 0.0f);
        CHECK(preSteerEnabled == false);
        AppMotion_GetTurnSpeed(&turnSpeed);
        CHECK(turnSpeed == 2200U);
        AppMotion_GetAckermann(&k, &ackermannEnabled);
        CHECK_FLOAT_EQ(k, 0.45f);
        CHECK(ackermannEnabled == false);
        AppMotion_GetTurnBrake(&lookaheadMs, &turnBrakeEnabled);
        CHECK_FLOAT_EQ(lookaheadMs, 10.0f);
        CHECK(turnBrakeEnabled == false);
    }

    AppMotion_Init();
    expect_turn_defaults();

    /* --- 3. Repeat once more from a different perturbed state, to
     * confirm this is deterministic restoration, not a fluke of the
     * particular values used above. -------------------------------- */
    AppMotion_SetPreSteer(999.0f, true);
    AppMotion_SetTurnSpeed(500U);
    AppMotion_SetAckermann(0.0f, true);
    AppMotion_SetTurnBrake(500.0f, true);

    AppMotion_Init();
    expect_turn_defaults();

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
