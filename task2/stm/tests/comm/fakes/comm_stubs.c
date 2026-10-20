/* Host-test stand-ins for everything Core/Src/app_comm.c needs beyond the
 * motion fakes (tests/motion/fakes/hw_stubs.c): the CMSIS message queues
 * (with fault injection), the kernel tick, the UART handle, a capturing
 * HAL_UART_Transmit, and the IMU/encoder snapshot getters.
 *
 * Single-threaded: taskENTER_CRITICAL()/EXIT are no-ops in the motion
 * fakes, and the test body plays the roles of CommTask and MotionTask
 * explicitly, in a fixed order. */

#include "cmsis_os.h"
#include "main.h"
#include "app_encoder.h"
#include "app_imu.h"
#include "test_fakes.h"

#include <stdlib.h>
#include <string.h>

/* --- message queue -------------------------------------------------- */

struct FakeOsQueue {
    uint8_t *storage;
    uint32_t capacity;
    uint32_t msgSize;
    uint32_t head;
    uint32_t count;
};

static osMessageQueueId_t faultQueue;
static FakeOsResetMode resetFaultMode = FAKEOS_RESET_OK;
static int resetFaultCount;
static int getFaultCount;
static uint32_t resetCallCount;

void FakeOs_SetFaultQueue(osMessageQueueId_t queue) { faultQueue = queue; }

void FakeOs_SetResetFault(FakeOsResetMode mode, int count)
{
    resetFaultMode = mode;
    resetFaultCount = count;
}

void FakeOs_SetGetFault(int count) { getFaultCount = count; }

void FakeOs_ClearFaults(void)
{
    resetFaultMode = FAKEOS_RESET_OK;
    resetFaultCount = 0;
    getFaultCount = 0;
}

uint32_t FakeOs_GetResetCallCount(void) { return resetCallCount; }

osMessageQueueId_t osMessageQueueNew(uint32_t msg_count, uint32_t msg_size, const void *attr)
{
    osMessageQueueId_t q = (osMessageQueueId_t)calloc(1U, sizeof(*q));

    (void)attr;
    if (q == NULL) { return NULL; }
    q->storage = (uint8_t *)calloc(msg_count, msg_size);
    q->capacity = msg_count;
    q->msgSize = msg_size;
    return q;
}

osStatus_t osMessageQueuePut(osMessageQueueId_t mq_id, const void *msg_ptr, uint8_t msg_prio, uint32_t timeout)
{
    uint32_t tail;

    (void)msg_prio; (void)timeout;
    if ((mq_id == NULL) || (msg_ptr == NULL)) { return osErrorParameter; }
    if (mq_id->count >= mq_id->capacity) { return osErrorResource; }
    tail = (mq_id->head + mq_id->count) % mq_id->capacity;
    memcpy(mq_id->storage + (size_t)tail * mq_id->msgSize, msg_ptr, mq_id->msgSize);
    mq_id->count++;
    return osOK;
}

osStatus_t osMessageQueueGet(osMessageQueueId_t mq_id, void *msg_ptr, uint8_t *msg_prio, uint32_t timeout)
{
    (void)msg_prio; (void)timeout;
    if ((mq_id == NULL) || (msg_ptr == NULL)) { return osErrorParameter; }

    if ((mq_id == faultQueue) && (getFaultCount != 0))
    {
        if (getFaultCount > 0) { getFaultCount--; }
        return osErrorResource;
    }

    if (mq_id->count == 0U) { return osErrorResource; }
    memcpy(msg_ptr, mq_id->storage + (size_t)mq_id->head * mq_id->msgSize, mq_id->msgSize);
    mq_id->head = (mq_id->head + 1U) % mq_id->capacity;
    mq_id->count--;
    return osOK;
}

uint32_t osMessageQueueGetCount(osMessageQueueId_t mq_id)
{
    return (mq_id == NULL) ? 0U : mq_id->count;
}

uint32_t osMessageQueueGetCapacity(osMessageQueueId_t mq_id)
{
    return (mq_id == NULL) ? 0U : mq_id->capacity;
}

osStatus_t osMessageQueueReset(osMessageQueueId_t mq_id)
{
    resetCallCount++;
    if (mq_id == NULL) { return osErrorParameter; }

    if ((mq_id == faultQueue) && (resetFaultMode != FAKEOS_RESET_OK) && (resetFaultCount != 0))
    {
        FakeOsResetMode mode = resetFaultMode;

        if (resetFaultCount > 0) { resetFaultCount--; }
        return (mode == FAKEOS_RESET_ERROR) ? osError : osOK;
    }

    mq_id->head = 0U;
    mq_id->count = 0U;
    return osOK;
}

uint32_t osKernelGetTickCount(void) { return Fake_GetTickCount(); }
uint32_t osKernelGetTickFreq(void) { return 1000U; }

/* --- queue handles app_comm.c expects from main.c ------------------- */

osMessageQueueId_t MotionQueueHandle;
osMessageQueueId_t CommRxQueueHandle;

/* --- UART ------------------------------------------------------------ */

USART_TypeDef fake_usart3_regs;
UART_HandleTypeDef huart3 = { &fake_usart3_regs, 0U };

static char txBuffer[COMM_TX_MAX];
static size_t txLength;

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    (void)huart; (void)Timeout;
    if ((txLength + Size) < sizeof(txBuffer))
    {
        memcpy(txBuffer + txLength, pData, Size);
        txLength += Size;
        txBuffer[txLength] = '\0';
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    (void)huart; (void)pData; (void)Size;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart)
{
    (void)huart;
    return HAL_OK;
}

void Error_Handler(void) { }

void Comm_TxClear(void)
{
    txLength = 0U;
    txBuffer[0] = '\0';
}

const char *Comm_TxText(void) { return txBuffer; }

unsigned Comm_TxLineCount(void)
{
    unsigned lines = 0U;
    size_t i;

    for (i = 0U; i < txLength; i++)
    {
        if (txBuffer[i] == '\n') { lines++; }
    }
    return lines;
}

/* Copies line `index` (without its newline) into out; returns 0 if absent. */
int Comm_TxLine(unsigned index, char *out, size_t outSize)
{
    size_t i = 0U;
    unsigned line = 0U;
    size_t start = 0U;

    for (i = 0U; i < txLength; i++)
    {
        if (txBuffer[i] == '\n')
        {
            if (line == index)
            {
                size_t n = i - start;

                if (n >= outSize) { n = outSize - 1U; }
                memcpy(out, txBuffer + start, n);
                out[n] = '\0';
                return 1;
            }
            line++;
            start = i + 1U;
        }
    }
    out[0] = '\0';
    return 0;
}

/* --- snapshot getters app_comm.c uses -------------------------------- */

AppEncoderSnapshot AppEncoder_GetSnapshot(void)
{
    AppEncoderSnapshot snapshot;

    snapshot.left = AppEncoder_Left();
    snapshot.right = AppEncoder_Right();
    snapshot.average = (snapshot.left + snapshot.right) / 2;
    return snapshot;
}

void AppIMU_GetSnapshot(AppIMUSnapshot *snapshot)
{
    if (snapshot != NULL)
    {
        snapshot->headingDeg = AppIMU_Heading();
        snapshot->gyroZDps = AppIMU_GyroZ();
        snapshot->headingSign = 1;
        snapshot->ready = AppIMU_IsReady() ? 1U : 0U;
    }
}
