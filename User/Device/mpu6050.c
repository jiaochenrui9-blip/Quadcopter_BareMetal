/**
  ******************************************************************************
  * @file    mpu6050.c
  * @brief   MPU6050 I2C DMA driver.
  *
  * The raw samples and converted physical values deliberately have file scope;
  * they cannot be accessed from other source files.
  ******************************************************************************
  */

#include "mpu6050.h"
#include "i2c.h"

#define MPU6050_I2C_ADDRESS           (0x68U << 1U)

#define MPU6050_REG_SMPLRT_DIV        0x19U
#define MPU6050_REG_CONFIG            0x1AU
#define MPU6050_REG_GYRO_CONFIG       0x1BU
#define MPU6050_REG_ACCEL_CONFIG      0x1CU
#define MPU6050_REG_INT_PIN_CFG       0x37U
#define MPU6050_REG_INT_ENABLE        0x38U
#define MPU6050_REG_ACCEL_XOUT_H      0x3BU
#define MPU6050_REG_PWR_MGMT_1        0x6BU
#define MPU6050_REG_WHO_AM_I          0x75U

/* Push-pull, active high, 50 us pulse, and any read clears the status flag so
 * it cannot latch up. Matches the rising-edge EXTI on MPU6050_INT_PIN. */
#define MPU6050_INT_PIN_CFG_VALUE     0x10U
#define MPU6050_INT_DATA_RDY_EN       0x01U

#define MPU6050_WHO_AM_I_VALUE        0x68U
#define MPU6050_SAMPLE_BYTES          14U

#define MPU6050_ACCEL_LSB_PER_G       4096.0F
#define MPU6050_GYRO_LSB_PER_DPS      65.5F
#define MPU6050_TEMP_LSB_PER_DEG_C    340.0F
#define MPU6050_TEMP_OFFSET_DEG_C     36.53F
#define MPU6050_GYRO_CALIBRATION_SAMPLE_COUNT 1000U

/* Measured while the airframe was placed level and stationary. */
#define MPU6050_ACCEL_X_OFFSET_G       0.104004F
#define MPU6050_ACCEL_Y_OFFSET_G       0.028441F
#define MPU6050_ACCEL_Z_OFFSET_G      -0.068359F

typedef struct
{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t temperature;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
} MPU6050_RawData_t;

static uint8_t mpu6050_rx_buffer[MPU6050_SAMPLE_BYTES];
static volatile MPU6050_RawData_t mpu6050_raw_data;
static volatile MPU6050_Data_t mpu6050_data;
static volatile uint32_t mpu6050_data_sequence;
static volatile uint8_t mpu6050_initialized;
static volatile uint8_t mpu6050_dma_busy;
static volatile float mpu6050_gyro_offset_dps[3];
static float mpu6050_gyro_calibration_sum_dps[3];
static volatile uint16_t mpu6050_gyro_calibration_count;
static volatile uint8_t mpu6050_gyro_calibration_active;

static HAL_StatusTypeDef MPU6050_WriteRegister(uint8_t reg, uint8_t value);
static int16_t MPU6050_CombineBytes(uint8_t msb, uint8_t lsb);
static void MPU6050_ConvertUnits(void);

HAL_StatusTypeDef MPU6050_Init(void)
{
  uint8_t who_am_i = 0U;
  uint8_t reset_command = 0x80U;
  uint8_t clock_source = 0x01U;
  uint8_t sample_divider = 0x00U;
  uint8_t low_pass_filter = 0x03U;
  uint8_t gyro_range_500_dps = 0x08U;
  uint8_t accel_range_8_g = 0x10U;

  mpu6050_initialized = 0U;
  mpu6050_dma_busy = 0U;

  if (MPU6050_WriteRegister(MPU6050_REG_PWR_MGMT_1, reset_command) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(100U);

  if (HAL_I2C_Mem_Read(&hi2c2, MPU6050_I2C_ADDRESS, MPU6050_REG_WHO_AM_I,
                       I2C_MEMADD_SIZE_8BIT, &who_am_i, 1U, 100U) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (who_am_i != MPU6050_WHO_AM_I_VALUE)
  {
    return HAL_ERROR;
  }

  if ((MPU6050_WriteRegister(MPU6050_REG_PWR_MGMT_1, clock_source) != HAL_OK) ||
      (MPU6050_WriteRegister(MPU6050_REG_SMPLRT_DIV, sample_divider) != HAL_OK) ||
      (MPU6050_WriteRegister(MPU6050_REG_CONFIG, low_pass_filter) != HAL_OK) ||
      (MPU6050_WriteRegister(MPU6050_REG_GYRO_CONFIG, gyro_range_500_dps) != HAL_OK) ||
      (MPU6050_WriteRegister(MPU6050_REG_ACCEL_CONFIG, accel_range_8_g) != HAL_OK))
  {
    return HAL_ERROR;
  }

  mpu6050_initialized = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_EnableDataReadyInterrupt(void)
{
  if (mpu6050_initialized == 0U)
  {
    return HAL_ERROR;
  }

  if (MPU6050_WriteRegister(MPU6050_REG_INT_PIN_CFG,
                            MPU6050_INT_PIN_CFG_VALUE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return MPU6050_WriteRegister(MPU6050_REG_INT_ENABLE,
                               MPU6050_INT_DATA_RDY_EN);
}

HAL_StatusTypeDef MPU6050_StartDmaRead(void)
{
  HAL_StatusTypeDef status;

  if ((mpu6050_initialized == 0U) || (mpu6050_dma_busy != 0U))
  {
    return HAL_BUSY;
  }

  mpu6050_dma_busy = 1U;
  status = HAL_I2C_Mem_Read_DMA(&hi2c2, MPU6050_I2C_ADDRESS,
                                MPU6050_REG_ACCEL_XOUT_H,
                                I2C_MEMADD_SIZE_8BIT, mpu6050_rx_buffer,
                                MPU6050_SAMPLE_BYTES);
  if (status != HAL_OK)
  {
    mpu6050_dma_busy = 0U;
  }

  return status;
}

void MPU6050_GetData(MPU6050_Data_t *data)
{
  uint32_t sequence_before;
  uint32_t sequence_after;

  if (data == NULL)
  {
    return;
  }

  /* An odd sequence means a write is in progress, so retry rather than copy.
   * A do-while cannot express that: continue would jump to the condition and
   * read sequence_after before it was ever assigned. */
  for (;;)
  {
    sequence_before = mpu6050_data_sequence;
    if ((sequence_before & 1U) != 0U)
    {
      continue;
    }

    data->acc_g[0] = mpu6050_data.acc_g[0];
    data->acc_g[1] = mpu6050_data.acc_g[1];
    data->acc_g[2] = mpu6050_data.acc_g[2];
    data->gyro_dps[0] = mpu6050_data.gyro_dps[0];
    data->gyro_dps[1] = mpu6050_data.gyro_dps[1];
    data->gyro_dps[2] = mpu6050_data.gyro_dps[2];
    data->temperature_deg_c = mpu6050_data.temperature_deg_c;
    sequence_after = mpu6050_data_sequence;
    if (sequence_before == sequence_after)
    {
      break;
    }
  }
} 

void MPU6050_GyroCalibrationStart(void)
{
  mpu6050_gyro_offset_dps[0] = 0.0F;
  mpu6050_gyro_offset_dps[1] = 0.0F;
  mpu6050_gyro_offset_dps[2] = 0.0F;
  mpu6050_gyro_calibration_sum_dps[0] = 0.0F;
  mpu6050_gyro_calibration_sum_dps[1] = 0.0F;
  mpu6050_gyro_calibration_sum_dps[2] = 0.0F;
  mpu6050_gyro_calibration_count = 0U;
  mpu6050_gyro_calibration_active = 1U;
}

uint8_t MPU6050_GyroCalibrationIsComplete(void)
{
  return (mpu6050_gyro_calibration_active == 0U) ? 1U : 0U;
}

void MPU6050_Update(void)
{
  mpu6050_raw_data.accel_x = MPU6050_CombineBytes(mpu6050_rx_buffer[0], mpu6050_rx_buffer[1]);
  mpu6050_raw_data.accel_y = MPU6050_CombineBytes(mpu6050_rx_buffer[2], mpu6050_rx_buffer[3]);
  mpu6050_raw_data.accel_z = MPU6050_CombineBytes(mpu6050_rx_buffer[4], mpu6050_rx_buffer[5]);
  mpu6050_raw_data.temperature = MPU6050_CombineBytes(mpu6050_rx_buffer[6], mpu6050_rx_buffer[7]);
  mpu6050_raw_data.gyro_x = MPU6050_CombineBytes(mpu6050_rx_buffer[8], mpu6050_rx_buffer[9]);
  mpu6050_raw_data.gyro_y = MPU6050_CombineBytes(mpu6050_rx_buffer[10], mpu6050_rx_buffer[11]);
  mpu6050_raw_data.gyro_z = MPU6050_CombineBytes(mpu6050_rx_buffer[12], mpu6050_rx_buffer[13]);

  mpu6050_data_sequence++;
  MPU6050_ConvertUnits();
  mpu6050_data_sequence++;
  mpu6050_dma_busy = 0U;
}

void MPU6050_I2CErrorCallback(void)
{
  mpu6050_dma_busy = 0U;
}

static HAL_StatusTypeDef MPU6050_WriteRegister(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c2, MPU6050_I2C_ADDRESS, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U, 100U);
}

static int16_t MPU6050_CombineBytes(uint8_t msb, uint8_t lsb)
{
  return (int16_t)(((uint16_t)msb << 8U) | (uint16_t)lsb);
}

static void MPU6050_ConvertUnits(void)
{
  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;

  mpu6050_data.acc_g[0] = ((float)mpu6050_raw_data.accel_x / MPU6050_ACCEL_LSB_PER_G)
                        - MPU6050_ACCEL_X_OFFSET_G;
  mpu6050_data.acc_g[1] = ((float)mpu6050_raw_data.accel_y / MPU6050_ACCEL_LSB_PER_G)
                        - MPU6050_ACCEL_Y_OFFSET_G;
  mpu6050_data.acc_g[2] = ((float)mpu6050_raw_data.accel_z / MPU6050_ACCEL_LSB_PER_G)
                        - MPU6050_ACCEL_Z_OFFSET_G;
  mpu6050_data.temperature_deg_c = ((float)mpu6050_raw_data.temperature / MPU6050_TEMP_LSB_PER_DEG_C) + MPU6050_TEMP_OFFSET_DEG_C;
  gyro_x_dps = (float)mpu6050_raw_data.gyro_x / MPU6050_GYRO_LSB_PER_DPS;
  gyro_y_dps = (float)mpu6050_raw_data.gyro_y / MPU6050_GYRO_LSB_PER_DPS;
  gyro_z_dps = (float)mpu6050_raw_data.gyro_z / MPU6050_GYRO_LSB_PER_DPS;

  if (mpu6050_gyro_calibration_active != 0U)
  {
    mpu6050_gyro_calibration_sum_dps[0] += gyro_x_dps;
    mpu6050_gyro_calibration_sum_dps[1] += gyro_y_dps;
    mpu6050_gyro_calibration_sum_dps[2] += gyro_z_dps;
    mpu6050_gyro_calibration_count++;

    if (mpu6050_gyro_calibration_count >= MPU6050_GYRO_CALIBRATION_SAMPLE_COUNT)
    {
      mpu6050_gyro_offset_dps[0] = mpu6050_gyro_calibration_sum_dps[0] / MPU6050_GYRO_CALIBRATION_SAMPLE_COUNT;
      mpu6050_gyro_offset_dps[1] = mpu6050_gyro_calibration_sum_dps[1] / MPU6050_GYRO_CALIBRATION_SAMPLE_COUNT;
      mpu6050_gyro_offset_dps[2] = mpu6050_gyro_calibration_sum_dps[2] / MPU6050_GYRO_CALIBRATION_SAMPLE_COUNT;
      mpu6050_gyro_calibration_active = 0U;
    }
  }

  mpu6050_data.gyro_dps[0] = gyro_x_dps - mpu6050_gyro_offset_dps[0];
  mpu6050_data.gyro_dps[1] = gyro_y_dps - mpu6050_gyro_offset_dps[1];
  mpu6050_data.gyro_dps[2] = gyro_z_dps - mpu6050_gyro_offset_dps[2];
}
