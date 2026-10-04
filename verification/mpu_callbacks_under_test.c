/* Extracted verbatim from main.c USER CODE 4 for host tests. */
#include "baremetal.h"
#include "mpu6050.h"
#include "i2c.h"
#include "OLED.h"

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c2) MPU6050_DmaCompleteCallback();
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1) OLED_I2CTxCpltCallback();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1) OLED_I2CErrorCallback();
  else if (hi2c == &hi2c2) MPU6050_ErrorCallback();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == MPU6050_INT_PIN)
  {
    ++g_imu_drdy_count;
    if (MPU6050_Ready != 0U) ++g_imu_coalesced_count;
    MPU6050_Ready = 1U;
  }
  else if (GPIO_Pin == REMOTE_NRF24_IRQ_PIN)
  {
    Remote_Ready = 1U;
  }
  else
  {
    /* No other EXTI line is in use. */
  }
}
