/* Compile-time check of the IR flag rules in Core/Inc/app_ir.h.
 *
 *   APP_IR_ENABLE     default 1 (production); =0 compiles the module out
 *   APP_IR_BENCH_LOG  default 0 (no unsolicited IRRAW); =1 with an explicit
 *                     APP_IR_ENABLE=0 is a compile error.
 *
 * Build each line (from repo root); each must compile and print "ok":
 *   gcc -std=c11 -Wall -Wextra -Werror -ICore/Inc -DEXPECT_ENABLE=1 -DEXPECT_LOG=0 tests/ir/app_ir_flags_test.c   (production default)
 *   gcc ... -DAPP_IR_ENABLE=0                     -DEXPECT_ENABLE=0 -DEXPECT_LOG=0 ...
 *   gcc ... -DAPP_IR_ENABLE=1                     -DEXPECT_ENABLE=1 -DEXPECT_LOG=0 ...
 *   gcc ... -DAPP_IR_BENCH_LOG=1                  -DEXPECT_ENABLE=1 -DEXPECT_LOG=1 ...
 *   gcc ... -DAPP_IR_ENABLE=1 -DAPP_IR_BENCH_LOG=1 -DEXPECT_ENABLE=1 -DEXPECT_LOG=1 ...
 * and each of these must FAIL to compile:
 *   gcc ... -DAPP_IR_ENABLE=0 -DAPP_IR_BENCH_LOG=1 ...
 *   gcc ... -DAPP_IR_ENABLE=2 ...
 *   gcc ... -DAPP_IR_BENCH_LOG=2 ...
 */

#include "app_ir.h"

#include <stdio.h>

#if !defined(EXPECT_ENABLE) || !defined(EXPECT_LOG)
#error "build with -DEXPECT_ENABLE=<0|1> -DEXPECT_LOG=<0|1>"
#endif

_Static_assert(APP_IR_ENABLE == EXPECT_ENABLE, "APP_IR_ENABLE resolved wrongly");
_Static_assert(APP_IR_BENCH_LOG == EXPECT_LOG, "APP_IR_BENCH_LOG resolved wrongly");

#if !APP_IR_ENABLE
/* Disabled: the header declares nothing of the module. */
#if defined(APP_IR_SERVICE_PERIOD_TICKS) || defined(APP_IR_FAULT_START)
#error "app_ir.h must declare nothing when APP_IR_ENABLE is 0"
#endif
#endif

int main(void)
{
    printf("ok: APP_IR_ENABLE=%d APP_IR_BENCH_LOG=%d\n",
           APP_IR_ENABLE, APP_IR_BENCH_LOG);
    return 0;
}
