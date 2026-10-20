/* Host-buildable regression test for AppMotion_GetMovementStatus(): the
 * read-only, atomic movement identity (state, command, movementSeq) plus the
 * measured filtered chassis speed that CommTask's USPARK detector consumes.
 *
 * Checks that a straight-forward movement is identifiable and distinct per
 * instance, that every other kind of motion is NOT mistaken for it, and that
 * the diagnostic speed is the encoder-measured, direction-normalised value
 * whether or not SPCTRL is enabled. (Atomicity itself -- one critical
 * section over all four fields -- is a code-structure property: there is no
 * concurrency in a host test.)
 *
 * Build (from repo root):
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -Itests/motion/fakes \
 *       -o tests/motion/app_motion_movement_status_test.exe \
 *       tests/motion/app_motion_movement_status_test.c Core/Src/app_motion.c \
 *       Core/Src/app_pid.c Core/Src/app_ir_edge.c Core/Src/app_ir_dist.c tests/motion/fakes/hw_stubs.c
 *   ./tests/motion/app_motion_movement_status_test.exe
 */

#include "app_motion.h"
#include "app_servo.h"
#include "test_fakes.h"

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

#define CHECK_EQ(actual, expected) do { \
    long a_ = (long)(actual); \
    long e_ = (long)(expected); \
    checks++; \
    if (a_ != e_) { \
        failures++; \
        printf("FAIL line %d: %s == %ld, expected %ld\n", \
               __LINE__, #actual, a_, e_); \
    } \
} while (0)

static AppMotionMovementStatus Status(void)
{
    AppMotionMovementStatus status;

    AppMotion_GetMovementStatus(&status);
    return status;
}

static void Tick(uint32_t tick)
{
    Fake_SetTickCount(tick);
    AppMotion_Update(0.01f);
}

static void SendStop(void)
{
    MotionCommand stopCommand = { MOTION_STOP, 0 };
    AppMotion_HandleCommand(&stopCommand);
}

static void ResetToIdle(void)
{
    SendStop();
    if (AppMotion_GetState() == APP_MOTION_BRAKING)
    {
        Tick(Fake_GetTickCount() + 200U);
    }
    CHECK(AppMotion_GetState() == APP_MOTION_IDLE);
}

static void Begin(uint32_t startTick)
{
    ResetToIdle();
    Fake_SetImuReady(true);
    Fake_SetTickCount(startTick);
    Fake_SetEncoderLeft(0);
    Fake_SetEncoderRight(0);
    Fake_SetHeading(0.0f);
    Fake_SetGyroZ(0.0f);
}

static void Start(MotionType type, int32_t value)
{
    MotionCommand command;

    command.type = type;
    command.value = value;
    AppMotion_HandleCommand(&command);
}

/* True only for what the detector protects. */
static int IsProtectedForward(const AppMotionMovementStatus *s)
{
    return (s->state == APP_MOTION_DISTANCE) &&
           (s->command == MOTION_FORWARD) &&
           (s->movementSeq != 0U);
}

static void test_idle_and_forward_identity(void)
{
    AppMotionMovementStatus idle;
    AppMotionMovementStatus first;
    AppMotionMovementStatus second;

    Begin(1000U);
    idle = Status();
    CHECK(idle.state == APP_MOTION_IDLE);
    CHECK(idle.command == MOTION_STOP);
    CHECK_EQ(idle.filteredSpeedCountsPerSec, 0);
    CHECK(!IsProtectedForward(&idle));

    Start(MOTION_FORWARD, 50);
    first = Status();
    CHECK(first.state == APP_MOTION_DISTANCE);
    CHECK(first.command == MOTION_FORWARD);
    CHECK(first.movementSeq != 0U);
    CHECK(first.movementSeq != idle.movementSeq);
    CHECK(IsProtectedForward(&first));

    /* Ticking does not change the identity. */
    Tick(1010U);
    Tick(1020U);
    CHECK_EQ(Status().movementSeq, first.movementSeq);

    /* STOP -> BRAKING (not a protected state), then IDLE. */
    SendStop();
    CHECK(Status().state == APP_MOTION_BRAKING);
    CHECK(!IsProtectedForward(&(AppMotionMovementStatus){
        Status().state, Status().command, Status().movementSeq, 0 }));
    Tick(Fake_GetTickCount() + 200U);
    CHECK(Status().state == APP_MOTION_IDLE);
    CHECK(Status().command == MOTION_STOP);

    /* A second forward movement is a different instance. */
    Start(MOTION_FORWARD, 50);
    second = Status();
    CHECK(IsProtectedForward(&second));
    CHECK_EQ(second.movementSeq, first.movementSeq + 1U);
}

static void test_normal_completion_leaves_protected_state(void)
{
    AppMotionMovementStatus before;

    Begin(2000U);
    Start(MOTION_FORWARD, 50);
    before = Status();
    CHECK(IsProtectedForward(&before));

    Fake_SetEncoderLeft(3850);                      /* 50 cm target */
    Fake_SetEncoderRight(3850);
    Tick(2100U);
    CHECK(Status().state == APP_MOTION_BRAKING);
    CHECK(!IsProtectedForward(&(AppMotionMovementStatus){
        Status().state, Status().command, Status().movementSeq, 0 }));

    Tick(Fake_GetTickCount() + 200U);
    CHECK(Status().state == APP_MOTION_IDLE);
}

static void test_not_forward(void)
{
    AppMotionMovementStatus s;
    MotionType turns[4] = {
        MOTION_FORWARD_LEFT, MOTION_FORWARD_RIGHT,
        MOTION_BACKWARD_LEFT, MOTION_BACKWARD_RIGHT
    };
    unsigned i;

    /* Reverse: DISTANCE, but BACKWARD. */
    Begin(3000U);
    Start(MOTION_BACKWARD, 50);
    s = Status();
    CHECK(s.state == APP_MOTION_DISTANCE);
    CHECK(s.command == MOTION_BACKWARD);
    CHECK(s.movementSeq != 0U);
    CHECK(!IsProtectedForward(&s));

    /* Each turn: PRE_STEER first (boot default), then TURN. */
    for (i = 0U; i < 4U; i++)
    {
        Begin(4000U + (i * 1000U));
        AppMotion_SetPreSteer(150.0f, true);
        Start(turns[i], 90);
        s = Status();
        CHECK(s.state == APP_MOTION_PRE_STEER);
        CHECK(s.command == turns[i]);
        CHECK(!IsProtectedForward(&s));

        Tick(Fake_GetTickCount() + 200U);
        s = Status();
        CHECK(s.state == APP_MOTION_TURN);
        CHECK(s.command == turns[i]);
        CHECK(!IsProtectedForward(&s));
    }

    /* Diagnostic. */
    Begin(10000U);
    Start(MOTION_PWM_TEST, 30);
    s = Status();
    CHECK(s.state == APP_MOTION_DIAGNOSTIC_PWM);
    CHECK(!IsProtectedForward(&s));

    /* MOV,F,0 completes synchronously and is never DISTANCE. */
    Begin(11000U);
    {
        AppMotionMovementStatus idle = Status();

        Start(MOTION_FORWARD, 0);
        s = Status();
        CHECK(s.state == APP_MOTION_IDLE);
        CHECK(!IsProtectedForward(&s));
        CHECK_EQ(s.movementSeq, idle.movementSeq);
    }
}

/* Counts per 10 ms tick -> counts/s through the 0.01 s Update(). */
static void DriveEncoders(int32_t perTick, unsigned ticks)
{
    unsigned i;
    int32_t position = 0;

    for (i = 0U; i < ticks; i++)
    {
        position += perTick;
        Fake_SetEncoderLeft(position);
        Fake_SetEncoderRight(position);
        Tick(Fake_GetTickCount() + 10U);
    }
}

static void test_filtered_speed(void)
{
    AppMotionMovementStatus s;

    /* Forward at 2000 counts/s: the EMA converges on it. SPCTRL (boot
     * default PRECISION) is enabled. */
    Begin(20000U);
    Start(MOTION_FORWARD, 200);
    CHECK_EQ(Status().filteredSpeedCountsPerSec, 0);
    DriveEncoders(20, 60);
    s = Status();
    CHECK(s.state == APP_MOTION_DISTANCE);
    CHECK(s.filteredSpeedCountsPerSec > 1800);
    CHECK(s.filteredSpeedCountsPerSec <= 2100);

    /* Same with SPCTRL disabled: the speed is still measured. */
    ResetToIdle();
    AppMotion_SetSpeedControl(4000.0f, 0.02f, 0.03f, false);
    Begin(30000U);
    Start(MOTION_FORWARD, 200);
    DriveEncoders(20, 60);
    s = Status();
    CHECK(s.filteredSpeedCountsPerSec > 1800);
    CHECK(s.filteredSpeedCountsPerSec <= 2100);

    /* A new movement starts from zero, not from the previous speed. */
    ResetToIdle();
    Begin(40000U);
    Start(MOTION_FORWARD, 200);
    CHECK_EQ(Status().filteredSpeedCountsPerSec, 0);

    /* Backward is direction-normalised: positive while reversing. */
    ResetToIdle();
    Begin(50000U);
    Start(MOTION_BACKWARD, 200);
    DriveEncoders(-20, 60);
    s = Status();
    CHECK(s.state == APP_MOTION_DISTANCE);
    CHECK(s.command == MOTION_BACKWARD);
    CHECK(s.filteredSpeedCountsPerSec > 1800);
}

int main(void)
{
    AppMotion_Init();

    test_idle_and_forward_identity();
    test_normal_completion_leaves_protected_state();
    test_not_forward();
    test_filtered_speed();

    AppMotion_GetMovementStatus(NULL);              /* must not crash */

    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0U) ? 0 : 1;
}
