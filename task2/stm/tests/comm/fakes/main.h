/* Host-test fake for main.h -- NOT the CubeMX header.
 *
 * Supplies only the HAL UART surface Core/Src/app_comm.c touches so the
 * real communication layer can be linked on the host: the UART handle,
 * HAL status codes, the error/flag masks and the transmit/receive entry
 * points (implemented in comm_stubs.c; HAL_UART_Transmit captures every
 * byte the firmware would put on the wire). Placed ahead of Core/Inc on
 * the include path. */
#ifndef FAKE_MAIN_H
#define FAKE_MAIN_H

#include <stdint.h>

typedef enum {
    HAL_OK      = 0x00U,
    HAL_ERROR   = 0x01U,
    HAL_BUSY    = 0x02U,
    HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;

typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
} USART_TypeDef;

typedef struct {
    USART_TypeDef *Instance;
    volatile uint32_t ErrorCode;
} UART_HandleTypeDef;

extern USART_TypeDef fake_usart3_regs;
#define USART3 (&fake_usart3_regs)

#define HAL_UART_ERROR_NONE 0x00000000U
#define HAL_UART_ERROR_PE   0x00000001U
#define HAL_UART_ERROR_NE   0x00000002U
#define HAL_UART_ERROR_FE   0x00000004U
#define HAL_UART_ERROR_ORE  0x00000008U

#define UART_FLAG_PE        0x00000001U
#define UART_FLAG_FE        0x00000002U
#define UART_FLAG_NE        0x00000004U
#define UART_FLAG_ORE       0x00000008U

#define READ_REG(REG) (REG)
#define __HAL_UART_CLEAR_PEFLAG(HANDLE) ((void)0)

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);

void Error_Handler(void);

/* --- test-only capture of everything the firmware transmits ---------- */

#include <stddef.h>

#define COMM_TX_MAX 8192U

void Comm_TxClear(void);
const char *Comm_TxText(void);
unsigned Comm_TxLineCount(void);
/* Copies line `index` (without its newline) into out; returns 0 if absent. */
int Comm_TxLine(unsigned index, char *out, size_t outSize);

#endif /* FAKE_MAIN_H */
