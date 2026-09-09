/* Extracted verbatim from main.c USER CODE 4 for host tests. */
#include "baremetal.h"
#include "mpu6050.h"
#include "i2c.h"


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c2)
  {
    MPU6050_DmaReady = 1U;
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c2)
  {
    MPU6050_I2CErrorCallback();
    MPU6050_Error = 1U;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == MPU6050_INT_PIN)
  {
    /* Start DMA here; main only consumes the completed frame. The driver
     * returns HAL_BUSY until the preceding frame has been parsed. */
    HAL_StatusTypeDef status = MPU6050_StartDmaRead();
    if ((status != HAL_OK) && (status != HAL_BUSY))
    {
      MPU6050_Error = 1U;
    }
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

