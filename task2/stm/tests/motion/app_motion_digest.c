/* Behaviour digest of the REAL Core/Src/app_motion.c (a tool, not a test).
 *
 * Runs a fixed set of ordinary movements -- MOV,F / MOV,B, all four turns,
 * zero and out-of-range values, IMU not ready, STOP during motion -- against a
 * deterministic toy plant (encoders follow the commanded motor duty, heading
 * follows the servo endpoint) and folds EVERY tick's observable behaviour
 * (motion state, motor duty, motor call counts, servo pulse, telemetry,
 * published events) into one 64-bit FNV-1a digest per scenario.
 *
 * Use: build it against two versions of app_motion.c and diff the output. A
 * behaviour-preserving change (a refactor, or new code that stays disarmed for
 * these commands) must give byte-identical output.
 *
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc -Itests/motion/fakes \
 *       -o tests/motion/app_motion_digest.exe tests/motion/app_motion_digest.c \
 *       Core/Src/app_motion.c Core/Src/app_pid.c Core/Src/app_ir_edge.c \n *       Core/Src/app_ir_dist.c tests/motion/fakes/hw_stubs.c
 *   ./tests/motion/app_motion_digest.exe
 *
 * With the Phase 5 IR edge code present, build with  -DDIGEST_IR_HOOK_DISARMED
 * to also call the (disarmed) IR service every tick, as MotionTask does.
 */

#include "app_motion.h"
#include "app_servo.h"
#include "test_fakes.h"

#include <stdio.h>
#include <string.h>

#ifdef DIGEST_IR_HOOK_DISARMED
#include "app_ir.h"
#if APP_IR_ENABLE
static AppIrSnapshot digestSnapshot;
#endif
#endif

static uint64_t hash;

static void Mix(uint64_t value)
{
    int i;

    for (i = 0; i < 8; i++)
    {
        hash ^= (value >> (8 * i)) & 0xFFU;
        hash *= 1099511628211ULL;
    }
}

static void MixS(int64_t value)
{
    Mix((uint64_t)value);
}

static uint32_t tick;
static int32_t encLeft, encRight;
static float heading;

static void Plant(void)
{
    int32_t l = Fake_GetLastMotorLeftDuty();
    int32_t r = Fake_GetLastMotorRightDuty();
    float servoOffset = (float)((int32_t)AppServo_GetUs() - 1530) / 530.0f;
    float before = heading;

    encLeft += l * 3 / 4;
    encRight += r * 3 / 4;

    /* turns: the servo endpoint steers, only while the wheels move */
    if ((l != 0) || (r != 0))
    {
        heading -= servoOffset * 0.9f * (float)(((l + r) > 0) ? 1 : -1);
    }

    Fake_SetEncoderLeft(encLeft);
    Fake_SetEncoderRight(encRight);
    Fake_SetHeading(heading);
    Fake_SetGyroZ((heading - before) / 0.01f);
}

static void Observe(void)
{
    AppMotionTelemetry t;
    AppMotionEvent e;

    AppMotion_GetTelemetry(&t);
    AppMotion_GetEvent(&e);

    Mix((uint64_t)AppMotion_GetState());
    MixS(Fake_GetLastMotorLeftDuty());
    MixS(Fake_GetLastMotorRightDuty());
    Mix(Fake_GetMotorSetCallCount());
    Mix(Fake_GetMotorStopCallCount());
    Mix(Fake_GetMotorBrakeCallCount());
    Mix(AppServo_GetUs());
    MixS(t.leftTargetCounts);
    MixS(t.rightTargetCounts);
    MixS(t.leftOutput);
    MixS(t.rightOutput);
    MixS(t.headingErrorX10);
    Mix(t.runSequence);
    Mix(t.active);
    Mix(t.movementBraking);
    Mix(t.movementPreSteering);
    Mix(e.sequence);
    Mix((uint64_t)e.type);
    Mix((uint64_t)e.command);
}

static void Step(void)
{
    tick += 10U;
    Fake_SetTickCount(tick);
    Plant();

#if defined(DIGEST_IR_HOOK_DISARMED) && APP_IR_ENABLE
    /* the disarmed IR service as MotionTask calls it (see main.c) */
    if (AppMotion_IrEdgeArmed())
    {
        AppMotion_ServiceIrEdge(&digestSnapshot, tick);
    }
#endif

    AppMotion_Update(0.01f);
    Observe();
}

static void Scenario(const char *name, MotionType type, int32_t value, int stopAtTick,
                     int imuReady)
{
    MotionCommand command;
    int i;

    /* start from idle with a clean world */
    {
        MotionCommand stop = { MOTION_STOP, 0 };

        AppMotion_HandleCommand(&stop);
        for (i = 0; (i < 40) && (AppMotion_GetState() != APP_MOTION_IDLE); i++)
        {
            Step();
        }
    }

    hash = 14695981039346656037ULL;
    encLeft = 0;
    encRight = 0;
    heading = 0.0f;
    Fake_SetImuReady(imuReady != 0);
    Fake_SetEncoderLeft(0);
    Fake_SetEncoderRight(0);
    Fake_SetHeading(0.0f);
    Fake_SetGyroZ(0.0f);
    Fake_ResetMotorCallCounts();

    command.type = type;
    command.value = value;
    AppMotion_HandleCommand(&command);
    Observe();

    for (i = 0; i < 1500; i++)
    {
        if (i == stopAtTick)
        {
            MotionCommand stop = { MOTION_STOP, 0 };

            AppMotion_HandleCommand(&stop);
            Observe();
        }

        Step();

        if ((AppMotion_GetState() == APP_MOTION_IDLE) && (i > 2) && (i > stopAtTick))
        {
            break;
        }
    }

    Mix((uint64_t)i);
    printf("%-22s %016llx  (%d ticks)\n", name, (unsigned long long)hash, i);
}

int main(void)
{
    AppMotion_Init();
    tick = 100000U;
    Fake_SetTickCount(tick);

    Scenario("F 20", MOTION_FORWARD, 20, -1, 1);
    Scenario("F 5", MOTION_FORWARD, 5, -1, 1);
    Scenario("F 60", MOTION_FORWARD, 60, -1, 1);
    Scenario("B 20", MOTION_BACKWARD, 20, -1, 1);
    Scenario("F 0", MOTION_FORWARD, 0, -1, 1);
    Scenario("B 0", MOTION_BACKWARD, 0, -1, 1);
    Scenario("F 1000 (bad)", MOTION_FORWARD, 1000, -1, 1);
    Scenario("F -1 (bad)", MOTION_FORWARD, -1, -1, 1);
    Scenario("F 20 imu not ready", MOTION_FORWARD, 20, -1, 0);
    Scenario("FL 90", MOTION_FORWARD_LEFT, 90, -1, 1);
    Scenario("FR 90", MOTION_FORWARD_RIGHT, 90, -1, 1);
    Scenario("BL 45", MOTION_BACKWARD_LEFT, 45, -1, 1);
    Scenario("BR 45", MOTION_BACKWARD_RIGHT, 45, -1, 1);
    Scenario("FL 0", MOTION_FORWARD_LEFT, 0, -1, 1);
    Scenario("F 60 stop@30", MOTION_FORWARD, 60, 30, 1);
    Scenario("B 60 stop@30", MOTION_BACKWARD, 60, 30, 1);
    Scenario("FL 90 stop@20", MOTION_FORWARD_LEFT, 90, 20, 1);
    Scenario("F 60 stop@0", MOTION_FORWARD, 60, 0, 1);
    Scenario("F 20 again", MOTION_FORWARD, 20, -1, 1);

    return 0;
}
