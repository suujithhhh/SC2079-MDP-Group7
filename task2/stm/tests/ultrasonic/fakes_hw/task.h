/* Host-test fake of task.h for the REAL Core/Src/app_ultrasonic_hw.c.
 * Critical sections are tracked (not just no-ops) so a test can prove how
 * many there are and that the register polling loop is not inside one. */
#ifndef FAKE_HW_TASK_H
#define FAKE_HW_TASK_H

void FakeCs_Enter(void);
void FakeCs_Exit(void);

#define taskENTER_CRITICAL() FakeCs_Enter()
#define taskEXIT_CRITICAL()  FakeCs_Exit()

#endif
