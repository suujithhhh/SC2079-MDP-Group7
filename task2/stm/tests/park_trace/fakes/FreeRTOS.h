/* Host-test fake for FreeRTOS.h used only by tests/park_trace: unlike
 * tests/motion/fakes/FreeRTOS.h, the critical-section macros here COUNT, so
 * the tests can prove every section is balanced and never nested by the trace
 * module itself. The real header is never on the include path of a host test. */
#ifndef PT_FAKE_FREERTOS_H
#define PT_FAKE_FREERTOS_H

#include <stdint.h>

extern int FakeCrit_Depth;          /* current nesting                       */
extern int FakeCrit_MaxDepth;       /* deepest nesting seen                  */
extern unsigned FakeCrit_Enters;    /* total enters                          */
extern unsigned FakeCrit_Underflows;/* exits without a matching enter        */

void FakeCrit_Enter(void);
void FakeCrit_Exit(void);

#define taskENTER_CRITICAL() FakeCrit_Enter()
#define taskEXIT_CRITICAL()  FakeCrit_Exit()

#endif /* PT_FAKE_FREERTOS_H */
