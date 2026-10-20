/* Host-test fake of FreeRTOS.h for the REAL Core/Src/app_ultrasonic_hw.c:
 * only the interrupt-priority constant it checks the NVIC against. */
#ifndef FAKE_HW_FREERTOS_H
#define FAKE_HW_FREERTOS_H

#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

#endif
