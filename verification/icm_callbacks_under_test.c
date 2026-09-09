#include "baremetal.h"
#include "icm42688.h"
#include "spi.h"
#include "i2c.h"
#include "OLED.h"


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi2) ICM42688_DmaCompleteCallback();
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi2) ICM42688_ErrorCallback();
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1) OLED_I2CTxCpltCallback();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1) OLED_I2CErrorCallback();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ICM42688_INT_PIN)
  {
    ++g_imu_drdy_count;
    if (ICM42688_Ready != 0U) ++g_imu_coalesced_count;
    ICM42688_Ready = 1U;
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
