/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   MPU6050 I2C DMA driver interface.
  ******************************************************************************
  */
#ifndef __MPU6050_H__
#define __MPU6050_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* DATA_RDY output of the MPU6050. Push-pull active high, so the EXTI triggers
 * on the rising edge. */
#define MPU6050_INT_PORT              GPIOB
#define MPU6050_INT_PIN               GPIO_PIN_0

typedef struct
{
  float acc_g[3];
  float gyro_dps[3];
  float temperature_deg_c;
} MPU6050_Data_t;

/* Initializes MPU6050 with accelerometer ±8 g and gyroscope ±500 °/s. */
HAL_StatusTypeDef MPU6050_Init(void);

/* Starts one 14-byte MPU6050 register read through I2C2 RX DMA. */
HAL_StatusTypeDef MPU6050_StartDmaRead(void);

/* Turns on the DATA_RDY output. Call it only once the consumer of the samples
 * exists, because the first completed read hands off through a semaphore. */
HAL_StatusTypeDef MPU6050_EnableDataReadyInterrupt(void);

/* Parses the completed DMA frame and updates converted MPU6050 data. */
void MPU6050_Update(void);
void MPU6050_I2CErrorCallback(void);
void MPU6050_GetData(MPU6050_Data_t *data);
void MPU6050_GyroCalibrationStart(void);
uint8_t MPU6050_GyroCalibrationIsComplete(void);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H__ */
