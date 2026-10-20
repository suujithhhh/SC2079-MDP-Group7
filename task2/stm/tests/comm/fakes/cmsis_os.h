/* Host-test fake for cmsis_os.h -- NOT a real CMSIS-RTOS port.
 *
 * Just enough of the CMSIS-RTOS v2 surface for Core/Src/app_comm.c to
 * compile and run single-threaded on the host: fixed-capacity message
 * queues (non-blocking; every timeout is ignored because app_comm.c only
 * ever passes 0 on its queue paths) and the kernel tick, which is the
 * motion fakes' tick clock (tests/motion/fakes, 1 tick == 1 ms).
 *
 * Fault injection (tests/comm/fakes/comm_stubs.c) lets a test make the
 * MotionQueue reset fail loudly, fail silently (return osOK but leave the
 * items in place, which is what the real CMSIS wrapper does when
 * xQueueReset() misbehaves) or make osMessageQueueGet() fail.
 *
 * Placed ahead of Core/Inc on the include path so this is picked up
 * instead of the real CMSIS header. */
#ifndef FAKE_CMSIS_OS_H
#define FAKE_CMSIS_OS_H

#include <stdint.h>
#include <stddef.h>

typedef enum {
    osOK                =  0,
    osError             = -1,
    osErrorTimeout      = -2,
    osErrorResource     = -3,
    osErrorParameter    = -4,
    osErrorNoMemory     = -5,
    osErrorISR          = -6
} osStatus_t;

typedef struct FakeOsQueue *osMessageQueueId_t;

osMessageQueueId_t osMessageQueueNew(uint32_t msg_count, uint32_t msg_size, const void *attr);
osStatus_t osMessageQueuePut(osMessageQueueId_t mq_id, const void *msg_ptr, uint8_t msg_prio, uint32_t timeout);
osStatus_t osMessageQueueGet(osMessageQueueId_t mq_id, void *msg_ptr, uint8_t *msg_prio, uint32_t timeout);
uint32_t osMessageQueueGetCount(osMessageQueueId_t mq_id);
uint32_t osMessageQueueGetCapacity(osMessageQueueId_t mq_id);
osStatus_t osMessageQueueReset(osMessageQueueId_t mq_id);

uint32_t osKernelGetTickCount(void);
uint32_t osKernelGetTickFreq(void);

/* --- test-only fault injection ------------------------------------- */

typedef enum {
    FAKEOS_RESET_OK = 0,        /* normal behaviour */
    FAKEOS_RESET_ERROR,         /* return osError, queue untouched */
    FAKEOS_RESET_SILENT_FAIL    /* return osOK, queue untouched */
} FakeOsResetMode;

/* Faults apply only to this queue (the MotionQueue); NULL disables them. */
void FakeOs_SetFaultQueue(osMessageQueueId_t queue);
/* The next `count` resets behave as `mode` (count < 0: until cleared). */
void FakeOs_SetResetFault(FakeOsResetMode mode, int count);
/* The next `count` gets return osErrorResource without consuming
 * (count < 0: until cleared). */
void FakeOs_SetGetFault(int count);
void FakeOs_ClearFaults(void);
uint32_t FakeOs_GetResetCallCount(void);

#endif /* FAKE_CMSIS_OS_H */
