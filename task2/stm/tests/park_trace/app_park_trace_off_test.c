/* Compile-time OFF isolation of the USPARK trace.
 *
 * Built WITHOUT any -DAPP_USPARK_TRACE (the production default), this proves:
 *   - the flag defaults to 0;
 *   - the header then declares nothing of the trace API (no types, no
 *     constants, no functions), so no production file can accidentally use it;
 *   - app_park_trace.c and app_park_trace_fmt.c compile to empty objects
 *     (checked on the object files by the test runner, with nm).
 * The test runner also checks that -DAPP_USPARK_TRACE=2 is rejected.
 *
 * Build (from repo root), note: NO -DAPP_USPARK_TRACE:
 *   gcc -std=c11 -Wall -Wextra -ICore/Inc \
 *       -o tests/park_trace/app_park_trace_off_test.exe \
 *       tests/park_trace/app_park_trace_off_test.c \
 *       Core/Src/app_park_trace.c Core/Src/app_park_trace_fmt.c
 */

#include "app_park_trace.h"

#include <stdio.h>

#if APP_USPARK_TRACE
#error "APP_USPARK_TRACE must default to 0"
#endif

#ifdef APP_PARK_TRACE_RING_SIZE
#error "the OFF header must not define the trace constants"
#endif

#ifdef APP_PARK_EV_US
#error "the OFF header must not define the trace events"
#endif

_Static_assert(APP_USPARK_TRACE == 0, "production default is OFF");

int main(void)
{
    /* The flag is a plain preprocessor 0: no run-time presence at all. */
    printf("1 checks, 0 failures (APP_USPARK_TRACE=%d)\n", APP_USPARK_TRACE);
    return 0;
}
