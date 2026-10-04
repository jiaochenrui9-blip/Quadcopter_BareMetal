#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"
#include "imu_data.h"

/* I2C2 on PB10 SCL / PB11 SDA (400 kHz, DMA1 Channel5 RX). AD0 is tied low on
 * the board, so the 7-bit address is 0x68. DATA_RDY leaves on the GYRO_INT net
 * (PB0, EXTI0, rising edge) as a push-pull active-high pulse. Pin names come
 * from the .ioc labels, so moving a pin there is picked up here. */
#define MPU6050_INT_PORT GYRO_INT_GPIO_Port
#define MPU6050_INT_PIN GYRO_INT_Pin
#define MPU6050_GYRO_CALIBRATION_SAMPLES 1000U
/* DLPF on (CONFIG=3) fixes the gyro output rate at 1 kHz; SMPLRT_DIV=0 keeps it. */
#define MPU6050_SAMPLE_PERIOD_S 0.001F

HAL_StatusTypeDef MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_EnableDataReadyInterrupt(void);
HAL_StatusTypeDef MPU6050_StartDmaRead(void);
uint8_t MPU6050_IsBusy(void);
/* Main context: consume completed buffer, or reset I2C2 on timeout/error.
 * HAL_BUSY = no new frame. No reuse until this function releases the buffer. */
HAL_StatusTypeDef MPU6050_Consume(IMU_Data_t *data);
void MPU6050_DmaCompleteCallback(void);
void MPU6050_ErrorCallback(void);
void MPU6050_GyroCalibrationStart(void);
uint8_t MPU6050_GyroCalibrationIsComplete(void);

#endif
