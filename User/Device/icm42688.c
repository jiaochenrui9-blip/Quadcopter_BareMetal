#include "icm42688.h"
#include "spi.h"
#include <string.h>

/* DS-000347 v1.9, bank 0. FIFO bypass, 16-bit sensor registers. */
#define REG_DEVICE_CONFIG 0x11U
#define REG_INT_CONFIG 0x14U
#define REG_FIFO_CONFIG 0x16U
#define REG_TEMP_DATA1 0x1DU
#define REG_INT_STATUS 0x2DU
#define REG_INTF_CONFIG0 0x4CU
#define REG_PWR_MGMT0 0x4EU
#define REG_GYRO_CONFIG0 0x4FU
#define REG_ACCEL_CONFIG0 0x50U
#define REG_GYRO_ACCEL_CONFIG0 0x52U
#define REG_FSYNC_CONFIG 0x62U
#define REG_INT_CONFIG1 0x64U
#define REG_INT_SOURCE0 0x65U
#define REG_WHO_AM_I 0x75U
#define REG_BANK_SEL 0x76U
#define SPI_TIMEOUT_MS 2U

static int32_t gyro_sum[3];
static float gyro_offset[3];
static uint32_t calibration_count;
static uint8_t calibration_active;
/* 0 idle, 1 DMA active, 2 complete/unconsumed, 3 error. */
static volatile uint8_t dma_state;
static uint8_t dma_tx[15];
static uint8_t dma_rx[15];
static uint32_t dma_started_ms;

static HAL_StatusTypeDef ReadRegisters(uint8_t reg, uint8_t *data, uint8_t count)
{
    uint8_t tx[15] = {0};
    uint8_t rx[15];
    HAL_StatusTypeDef status;
    if ((count == 0U) || (count > 14U)) return HAL_ERROR;
    tx[0] = reg | 0x80U;
    HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, (uint16_t)count + 1U, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);
    if (status == HAL_OK) memcpy(data, &rx[1], count);
    return status;
}

static HAL_StatusTypeDef WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};
    uint8_t rx[2];
    HAL_StatusTypeDef status;
    HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_RESET);
    status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2U, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);
    return status;
}

static HAL_StatusTypeDef Configure(uint8_t reg, uint8_t value)
{
    uint8_t actual;
    if (WriteRegister(reg, value) != HAL_OK) return HAL_ERROR;
    if (ReadRegisters(reg, &actual, 1U) != HAL_OK) return HAL_ERROR;
    return (actual == value) ? HAL_OK : HAL_ERROR;
}

void ICM42688_GyroCalibrationStart(void)
{
    memset(gyro_sum, 0, sizeof(gyro_sum));
    memset(gyro_offset, 0, sizeof(gyro_offset));
    calibration_count = 0U;
    calibration_active = 1U;
}

uint8_t ICM42688_GyroCalibrationIsComplete(void)
{
    return (calibration_active == 0U &&
            calibration_count == ICM42688_GYRO_CALIBRATION_SAMPLES);
}

HAL_StatusTypeDef ICM42688_Init(void)
{
    uint8_t id;
    dma_state = 0U;
    HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(10U);
    if (WriteRegister(REG_BANK_SEL, 0U) != HAL_OK) return HAL_ERROR;
    if (ReadRegisters(REG_WHO_AM_I, &id, 1U) != HAL_OK || id != 0x47U) return HAL_ERROR;
    if (WriteRegister(REG_DEVICE_CONFIG, 1U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(2U); /* Datasheet requires at least 1 ms after soft reset. */
    if (ReadRegisters(REG_INT_STATUS, &id, 1U) != HAL_OK || !(id & 0x10U)) return HAL_ERROR;
    if (ReadRegisters(REG_WHO_AM_I, &id, 1U) != HAL_OK || id != 0x47U) return HAL_ERROR;

    /* Retain reserved reset bits; big endian, disable I2C for SPI use. */
    if (Configure(REG_INT_SOURCE0, 0x00U) != HAL_OK ||
        Configure(REG_FIFO_CONFIG, 0x00U) != HAL_OK ||
        Configure(REG_INTF_CONFIG0, 0x33U) != HAL_OK ||
        Configure(REG_FSYNC_CONFIG, 0x00U) != HAL_OK ||
        Configure(REG_GYRO_CONFIG0, 0x46U) != HAL_OK || /* 500 dps, 1 kHz */
        Configure(REG_ACCEL_CONFIG0, 0x26U) != HAL_OK || /* 8 g, 1 kHz */
        Configure(REG_GYRO_ACCEL_CONFIG0, 0x44U) != HAL_OK || /* UI LPF 100 Hz */
        Configure(REG_INT_CONFIG, 0x03U) != HAL_OK || /* INT1 high, push-pull, pulse */
        Configure(REG_INT_CONFIG1, 0x00U) != HAL_OK) /* Clear INT_ASYNC_RESET */
        return HAL_ERROR;

    if (WriteRegister(REG_PWR_MGMT0, 0x0FU) != HAL_OK) return HAL_ERROR;
    /* No writes for >=200 us after OFF->LN. Also allow gyro startup. */
    HAL_Delay(50U);
    if (ReadRegisters(REG_PWR_MGMT0, &id, 1U) != HAL_OK || id != 0x0FU) return HAL_ERROR;
    ICM42688_GyroCalibrationStart();
    return HAL_OK;
}

HAL_StatusTypeDef ICM42688_EnableDataReadyInterrupt(void)
{
    uint8_t status;
    if (ReadRegisters(REG_INT_STATUS, &status, 1U) != HAL_OK) return HAL_ERROR;
    return Configure(REG_INT_SOURCE0, 0x08U);
}

static int16_t SignedBE(const uint8_t *p)
{
    uint16_t value = ((uint16_t)p[0] << 8) | p[1];
    return (int16_t)((int32_t)value - ((value & 0x8000U) ? 65536 : 0));
}

uint8_t ICM42688_IsBusy(void)
{
    return dma_state != 0U;
}

HAL_StatusTypeDef ICM42688_StartDmaRead(void)
{
    uint8_t status;
    HAL_StatusTypeDef result;
    if (dma_state != 0U) return HAL_BUSY;
    /* Reading status prevents duplicate calibration/control on a stale flag. */
    result = ReadRegisters(REG_INT_STATUS, &status, 1U);
    if (result != HAL_OK) return result;
    if (!(status & 0x08U)) return HAL_BUSY;
    memset(dma_tx, 0, sizeof(dma_tx));
    dma_tx[0] = REG_TEMP_DATA1 | 0x80U;
    dma_started_ms = HAL_GetTick();
    dma_state = 1U;
    HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_RESET);
    result = HAL_SPI_TransmitReceive_DMA(&hspi2, dma_tx, dma_rx, sizeof(dma_tx));
    if (result != HAL_OK)
    {
        /* HAL may already have started one DMA channel: clean both in main. */
        dma_state = 3U;
        return HAL_ERROR;
    }
    return HAL_OK;
}

void ICM42688_DmaCompleteCallback(void)
{
    if (dma_state == 1U)
    {
        HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);
        dma_state = 2U;
    }
}

void ICM42688_ErrorCallback(void)
{
    if (dma_state == 1U) dma_state = 3U;
}

HAL_StatusTypeDef ICM42688_Consume(IMU_Data_t *data)
{
    const uint8_t *raw = &dma_rx[1];
    int16_t accel[3], gyro[3];
    IMU_Data_t sample;
    if (data == NULL) return HAL_ERROR;
    if (dma_state == 3U || ((dma_state == 1U || dma_state == 2U) &&
        (uint32_t)(HAL_GetTick() - dma_started_ms) >= 5U))
    {
        (void)HAL_SPI_Abort(&hspi2);
        HAL_GPIO_WritePin(ICM42688_CS_PORT, ICM42688_CS_PIN, GPIO_PIN_SET);
        dma_state = 0U;
        return HAL_ERROR;
    }
    if (dma_state != 2U) return HAL_BUSY;
    /* Only main starts DMA, so releasing here cannot overwrite this buffer
     * during parsing. EXTI merely sets the application flag. */
    dma_state = 0U;
    for (uint8_t i = 0; i < 3U; ++i)
    {
        accel[i] = SignedBE(&raw[2U + 2U * i]);
        gyro[i] = SignedBE(&raw[8U + 2U * i]);
        if (accel[i] == INT16_MIN || gyro[i] == INT16_MIN) return HAL_ERROR;
    }
    if (calibration_active)
    {
        for (uint8_t i = 0; i < 3U; ++i) gyro_sum[i] += gyro[i];
        if (++calibration_count == ICM42688_GYRO_CALIBRATION_SAMPLES)
        {
            for (uint8_t i = 0; i < 3U; ++i)
                gyro_offset[i] = (float)gyro_sum[i] /
                    ((float)ICM42688_GYRO_CALIBRATION_SAMPLES * 65.536F);
            calibration_active = 0U;
        }
    }
    for (uint8_t i = 0; i < 3U; ++i)
    {
        /* Native chip axes for now; map to body axes after PCB orientation is known.
         * Do not carry the old MPU6050's measured accelerometer offsets across. */
        sample.acc_g[i] = (float)accel[i] / 4096.0F;
        sample.gyro_dps[i] = (float)gyro[i] / 65.536F - gyro_offset[i];
    }
    sample.temperature_deg_c = (float)SignedBE(raw) / 132.48F + 25.0F;
    *data = sample;
    return HAL_OK;
}
