#ifndef ICM42688_H
#define ICM42688_H

#include "main.h"
#include "imu_data.h"

/* SPI2 mode 0: PB13 SCK, PB14 MISO, PB15 MOSI, 4.5 MHz.
 * DMA1 Channel4 RX / Channel5 TX. INT1 is push-pull active-high. */
#define ICM42688_CS_PORT GPIOB
#define ICM42688_CS_PIN GPIO_PIN_12
#define ICM42688_INT_PORT GPIOB
#define ICM42688_INT_PIN GPIO_PIN_0
#define ICM42688_GYRO_CALIBRATION_SAMPLES 1000U
#define ICM42688_SAMPLE_PERIOD_S 0.001F

HAL_StatusTypeDef ICM42688_Init(void);
HAL_StatusTypeDef ICM42688_EnableDataReadyInterrupt(void);
HAL_StatusTypeDef ICM42688_StartDmaRead(void);
uint8_t ICM42688_IsBusy(void);
/* Main context: consume completed buffer, or abort on timeout/error.
 * HAL_BUSY = no completed frame. No reuse until this function releases it. */
HAL_StatusTypeDef ICM42688_Consume(IMU_Data_t *data);
void ICM42688_DmaCompleteCallback(void);
void ICM42688_ErrorCallback(void);
void ICM42688_GyroCalibrationStart(void);
uint8_t ICM42688_GyroCalibrationIsComplete(void);

#endif
