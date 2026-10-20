#include "icm20948.h"

/* ICM-20948 can use 0x68 or 0x69 depending on AD0.  The driver probes both
 * addresses so the project is not tied to one board strap. */
#define ICM_ADDR0_7BIT       0x68U
#define ICM_ADDR1_7BIT       0x69U
#define ICM_I2C_TIMEOUT_MS   100U

/* ICM-20948 user-bank registers used here. */
#define REG_BANK_SEL         0x7FU
#define WHO_AM_I_REG         0x00U
#define PWR_MGMT_1           0x06U
#define PWR_MGMT_2           0x07U
#define GYRO_ZOUT_H          0x37U

/* User bank 2. */
#define GYRO_SMPLRT_DIV      0x00U
#define GYRO_CONFIG_1        0x01U

/* +/-500 dps => 65.5 LSB/(deg/s). */
#define GYRO_SENS_500DPS     65.5f

static I2C_HandleTypeDef *icm_i2c = NULL;
static uint16_t icm_addr_hal = 0U;   /* HAL expects the 7-bit address shifted left by 1. */
static uint8_t icm_addr_7bit = 0U;
static float gyro_z_bias_dps = 0.0f;
static uint8_t cached_who_am_i = 0U;
static uint8_t current_bank = 0xFFU;

static HAL_StatusTypeDef write_reg_raw(uint8_t reg, uint8_t value)
{
    if ((icm_i2c == NULL) || (icm_addr_hal == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(icm_i2c,
                             icm_addr_hal,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             ICM_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef read_regs_raw(uint8_t reg, uint8_t *data, uint16_t len)
{
    if ((icm_i2c == NULL) || (icm_addr_hal == 0U) || (data == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(icm_i2c,
                            icm_addr_hal,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            len,
                            ICM_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef select_bank(uint8_t bank)
{
    uint8_t reg_value;

    if (current_bank == bank)
    {
        return HAL_OK;
    }

    reg_value = (uint8_t)((bank & 0x03U) << 4U);
    if (write_reg_raw(REG_BANK_SEL, reg_value) != HAL_OK)
    {
        return HAL_ERROR;
    }

    current_bank = bank;
    return HAL_OK;
}

static HAL_StatusTypeDef write_reg(uint8_t bank, uint8_t reg, uint8_t value)
{
    if (select_bank(bank) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return write_reg_raw(reg, value);
}

static HAL_StatusTypeDef read_regs(uint8_t bank, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (select_bank(bank) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return read_regs_raw(reg, data, len);
}

static HAL_StatusTypeDef probe_address(uint8_t addr_7bit, uint8_t *who)
{
    HAL_StatusTypeDef status;

    if ((icm_i2c == NULL) || (who == NULL))
    {
        return HAL_ERROR;
    }

    icm_addr_7bit = addr_7bit;
    icm_addr_hal = (uint16_t)(addr_7bit << 1U);
    current_bank = 0xFFU;

    status = HAL_I2C_IsDeviceReady(icm_i2c, icm_addr_hal, 3U, ICM_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Explicitly select bank 0 before reading WHO_AM_I. */
    if (write_reg_raw(REG_BANK_SEL, 0x00U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    current_bank = 0U;

    if (read_regs_raw(WHO_AM_I_REG, who, 1U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return (*who == ICM20948_EXPECTED_WHO_AM_I) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef ICM20948_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who = 0U;

    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }

    icm_i2c = hi2c;
    icm_addr_hal = 0U;
    icm_addr_7bit = 0U;
    current_bank = 0xFFU;
    cached_who_am_i = 0U;
    gyro_z_bias_dps = 0.0f;

    /* Probe both legal ICM-20948 addresses. */
    if (probe_address(ICM_ADDR0_7BIT, &who) != HAL_OK)
    {
        if (probe_address(ICM_ADDR1_7BIT, &who) != HAL_OK)
        {
            icm_addr_hal = 0U;
            icm_addr_7bit = 0U;
            return HAL_ERROR;
        }
    }

    cached_who_am_i = who;

    /* Device reset, then select auto clock and enable all sensor axes. */
    if (write_reg(0U, PWR_MGMT_1, 0x80U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(100U);
    current_bank = 0xFFU;

    /* Wait until the device is responding again after reset. */
    if (HAL_I2C_IsDeviceReady(icm_i2c, icm_addr_hal, 5U, ICM_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (write_reg(0U, PWR_MGMT_1, 0x01U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (write_reg(0U, PWR_MGMT_2, 0x00U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    HAL_Delay(20U);

    /* Gyroscope: sample divider 4, +/-500 dps, DLPF enabled.
     * GYRO_CONFIG_1 = 0x33 gives FS_SEL=1 -> +/-500 dps. */
    if (write_reg(2U, GYRO_SMPLRT_DIV, 0x04U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (write_reg(2U, GYRO_CONFIG_1, 0x33U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (select_bank(0U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef read_gyro_z_uncalibrated(float *dps)
{
    uint8_t raw[2];
    int16_t z;

    if (dps == NULL)
    {
        return HAL_ERROR;
    }

    if (read_regs(0U, GYRO_ZOUT_H, raw, 2U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    z = (int16_t)(((uint16_t)raw[0] << 8U) | (uint16_t)raw[1]);
    *dps = (float)z / GYRO_SENS_500DPS;
    return HAL_OK;
}

HAL_StatusTypeDef ICM20948_CalibrateGyroZ(uint16_t samples, uint16_t sample_delay_ms)
{
    uint16_t i;
    float sum = 0.0f;
    float value;

    if ((samples == 0U) || (icm_i2c == NULL))
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < samples; ++i)
    {
        if (read_gyro_z_uncalibrated(&value) != HAL_OK)
        {
            return HAL_ERROR;
        }

        sum += value;
        HAL_Delay(sample_delay_ms);
    }

    gyro_z_bias_dps = sum / (float)samples;
    return HAL_OK;
}

HAL_StatusTypeDef ICM20948_ReadGyroZ(float *dps_bias_corrected)
{
    float raw_dps;

    if (dps_bias_corrected == NULL)
    {
        return HAL_ERROR;
    }

    if (read_gyro_z_uncalibrated(&raw_dps) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *dps_bias_corrected = raw_dps - gyro_z_bias_dps;
    return HAL_OK;
}

uint8_t ICM20948_GetWhoAmI(void)
{
    return cached_who_am_i;
}

uint8_t ICM20948_GetAddress7Bit(void)
{
    return icm_addr_7bit;
}

float ICM20948_GetGyroZBias(void)
{
    return gyro_z_bias_dps;
}
