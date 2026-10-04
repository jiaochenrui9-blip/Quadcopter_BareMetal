#include "mpu6050.h"
#include "i2c.h"
#include <string.h>

/* MPU-6000/MPU-6050 register map, rev 4.2. */
#define MPU6050_ADDRESS (0x68U << 1U)
#define REG_SMPLRT_DIV 0x19U
#define REG_CONFIG 0x1AU
#define REG_GYRO_CONFIG 0x1BU
#define REG_ACCEL_CONFIG 0x1CU
#define REG_INT_PIN_CFG 0x37U
#define REG_INT_ENABLE 0x38U
#define REG_INT_STATUS 0x3AU
#define REG_PWR_MGMT_1 0x6BU
#define REG_WHO_AM_I 0x75U
#define WHO_AM_I_VALUE 0x68U
#define INT_STATUS_DATA_RDY 0x01U
#define I2C_TIMEOUT_MS 100U
#define DMA_TIMEOUT_MS 5U

/* One burst: INT_STATUS (0x3A), then ACCEL_XOUT_H..GYRO_ZOUT_L (0x3B..0x48).
 * Reading the status inside the burst tells Consume whether the sample is new
 * without a second I2C transaction in the main loop. */
#define FRAME_BYTES 15U
#define FRAME_ACCEL 1U
#define FRAME_TEMP 7U
#define FRAME_GYRO 9U

#define ACCEL_LSB_PER_G 4096.0F /* +/-8 g */
#define GYRO_LSB_PER_DPS 65.5F /* +/-500 dps */
#define TEMP_LSB_PER_DEG_C 340.0F
#define TEMP_OFFSET_DEG_C 36.53F

/* Measured earlier with the airframe level and stationary. */
#define ACCEL_X_OFFSET_G 0.104004F
#define ACCEL_Y_OFFSET_G 0.028441F
#define ACCEL_Z_OFFSET_G (-0.068359F)

static int32_t gyro_sum[3];
static float gyro_offset[3];
static uint32_t calibration_count;
static uint8_t calibration_active;
/* 0 idle, 1 DMA active, 2 complete/unconsumed, 3 error. */
static volatile uint8_t dma_state;
static uint8_t dma_rx[FRAME_BYTES];
static uint32_t dma_started_ms;

static HAL_StatusTypeDef ReadRegister(uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,
                            value, 1U, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef WriteRegister(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1U, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef Configure(uint8_t reg, uint8_t value)
{
    uint8_t actual;
    if (WriteRegister(reg, value) != HAL_OK) return HAL_ERROR;
    if (ReadRegister(reg, &actual) != HAL_OK) return HAL_ERROR;
    return (actual == value) ? HAL_OK : HAL_ERROR;
}

static int16_t SignedBE(const uint8_t *p)
{
    uint16_t value = ((uint16_t)p[0] << 8) | p[1];
    return (int16_t)((int32_t)value - ((value & 0x8000U) ? 65536 : 0));
}

/* Re-initialising I2C2 stops a running DMA (MspDeInit) and clears a stuck HAL
 * state, the same recovery the OLED link uses on I2C1. */
static void ResetBus(void)
{
    (void)HAL_I2C_DeInit(&hi2c2);
    (void)HAL_I2C_Init(&hi2c2);
}

void MPU6050_GyroCalibrationStart(void)
{
    memset(gyro_sum, 0, sizeof(gyro_sum));
    memset(gyro_offset, 0, sizeof(gyro_offset));
    calibration_count = 0U;
    calibration_active = 1U;
}

uint8_t MPU6050_GyroCalibrationIsComplete(void)
{
    return (calibration_active == 0U &&
            calibration_count == MPU6050_GYRO_CALIBRATION_SAMPLES);
}

HAL_StatusTypeDef MPU6050_Init(void)
{
    uint8_t id;
    dma_state = 0U;
    if (WriteRegister(REG_PWR_MGMT_1, 0x80U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(100U); /* Register map: allow 100 ms for the device reset. */
    if (ReadRegister(REG_WHO_AM_I, &id) != HAL_OK || id != WHO_AM_I_VALUE) return HAL_ERROR;

    if (Configure(REG_PWR_MGMT_1, 0x01U) != HAL_OK || /* Wake, PLL on gyro X */
        Configure(REG_SMPLRT_DIV, 0x00U) != HAL_OK ||
        Configure(REG_CONFIG, 0x03U) != HAL_OK || /* DLPF 42 Hz gyro, 1 kHz rate */
        Configure(REG_GYRO_CONFIG, 0x08U) != HAL_OK || /* 500 dps */
        Configure(REG_ACCEL_CONFIG, 0x10U) != HAL_OK) /* 8 g */
        return HAL_ERROR;
    MPU6050_GyroCalibrationStart();
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_EnableDataReadyInterrupt(void)
{
    uint8_t status;
    /* Any read clears INT_STATUS (INT_RD_CLEAR), so drain a stale bit first. */
    if (ReadRegister(REG_INT_STATUS, &status) != HAL_OK) return HAL_ERROR;
    /* INT: push-pull, active high, 50 us pulse, then DATA_RDY enabled. */
    if (Configure(REG_INT_PIN_CFG, 0x10U) != HAL_OK) return HAL_ERROR;
    return Configure(REG_INT_ENABLE, 0x01U);
}

uint8_t MPU6050_IsBusy(void)
{
    return dma_state != 0U;
}

HAL_StatusTypeDef MPU6050_StartDmaRead(void)
{
    if (dma_state != 0U) return HAL_BUSY;
    dma_started_ms = HAL_GetTick();
    dma_state = 1U;
    if (HAL_I2C_Mem_Read_DMA(&hi2c2, MPU6050_ADDRESS, REG_INT_STATUS,
                             I2C_MEMADD_SIZE_8BIT, dma_rx, FRAME_BYTES) != HAL_OK)
    {
        /* HAL may already have claimed the bus: reset it in main. */
        dma_state = 3U;
        return HAL_ERROR;
    }
    return HAL_OK;
}

void MPU6050_DmaCompleteCallback(void)
{
    if (dma_state == 1U) dma_state = 2U;
}

void MPU6050_ErrorCallback(void)
{
    if (dma_state == 1U) dma_state = 3U;
}

HAL_StatusTypeDef MPU6050_Consume(IMU_Data_t *data)
{
    int16_t accel[3], gyro[3];
    IMU_Data_t sample;
    if (data == NULL) return HAL_ERROR;
    if (dma_state == 3U || ((dma_state == 1U || dma_state == 2U) &&
        (uint32_t)(HAL_GetTick() - dma_started_ms) >= DMA_TIMEOUT_MS))
    {
        ResetBus();
        dma_state = 0U;
        return HAL_ERROR;
    }
    if (dma_state != 2U) return HAL_BUSY;
    /* Only main starts DMA, so releasing here cannot overwrite this buffer
     * during parsing. EXTI merely sets the application flag. */
    dma_state = 0U;
    /* A repeated EXTI flag must not count the same sample twice. */
    if (!(dma_rx[0] & INT_STATUS_DATA_RDY)) return HAL_BUSY;
    for (uint8_t i = 0; i < 3U; ++i)
    {
        accel[i] = SignedBE(&dma_rx[FRAME_ACCEL + 2U * i]);
        gyro[i] = SignedBE(&dma_rx[FRAME_GYRO + 2U * i]);
        if (accel[i] == INT16_MIN || gyro[i] == INT16_MIN) return HAL_ERROR;
    }
    if (calibration_active)
    {
        for (uint8_t i = 0; i < 3U; ++i) gyro_sum[i] += gyro[i];
        if (++calibration_count == MPU6050_GYRO_CALIBRATION_SAMPLES)
        {
            for (uint8_t i = 0; i < 3U; ++i)
                gyro_offset[i] = (float)gyro_sum[i] /
                    ((float)MPU6050_GYRO_CALIBRATION_SAMPLES * GYRO_LSB_PER_DPS);
            calibration_active = 0U;
        }
    }
    /* Native chip axes, as before; map to body axes once the PCB orientation
     * is checked on the bench. */
    sample.acc_g[0] = (float)accel[0] / ACCEL_LSB_PER_G - ACCEL_X_OFFSET_G;
    sample.acc_g[1] = (float)accel[1] / ACCEL_LSB_PER_G - ACCEL_Y_OFFSET_G;
    sample.acc_g[2] = (float)accel[2] / ACCEL_LSB_PER_G - ACCEL_Z_OFFSET_G;
    for (uint8_t i = 0; i < 3U; ++i)
        sample.gyro_dps[i] = (float)gyro[i] / GYRO_LSB_PER_DPS - gyro_offset[i];
    sample.temperature_deg_c = (float)SignedBE(&dma_rx[FRAME_TEMP]) / TEMP_LSB_PER_DEG_C +
                               TEMP_OFFSET_DEG_C;
    *data = sample;
    return HAL_OK;
}
