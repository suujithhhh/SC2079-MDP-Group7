#ifndef ICM20948_H
#define ICM20948_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* On this project the onboard IMU bus is owned by CubeMX:
 *   PB10 = I2C2_SCL
 *   PB11 = I2C2_SDA
 * The driver therefore uses the generated I2C_HandleTypeDef instead of
 * reconfiguring/toggling the pins itself. */
#define ICM20948_EXPECTED_WHO_AM_I 0xEAU

HAL_StatusTypeDef ICM20948_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef ICM20948_CalibrateGyroZ(uint16_t samples, uint16_t sample_delay_ms);
HAL_StatusTypeDef ICM20948_ReadGyroZ(float *dps_bias_corrected);
uint8_t ICM20948_GetWhoAmI(void);
uint8_t ICM20948_GetAddress7Bit(void);
float ICM20948_GetGyroZBias(void);

#endif /* ICM20948_H */
