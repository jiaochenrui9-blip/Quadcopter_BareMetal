/**
  ******************************************************************************
  * @file    imu_task.h
  * @brief   IMU application data interface.
  ******************************************************************************
  */
#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "attitude.h"

extern IMU_Data_t g_imu_data;
extern Attitude_Data_t g_attitude_data;
extern volatile uint8_t g_gyro_calibrated;


#ifdef __cplusplus
}
#endif

#endif /* __IMU_TASK_H__ */
