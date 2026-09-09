/**
  ******************************************************************************
  * @file    attitude.h
  * @brief   Six-axis Mahony quaternion attitude update.
  ******************************************************************************
  */
#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "imu_data.h"

typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;

  float roll_deg;
  float pitch_deg;
  float yaw_deg;

  float gyro_corrected_dps[3];
  float integral_error[3];
} Attitude_Data_t;

void Attitude_Init(void);

/* Seeds roll and pitch from a stationary gravity reading so the filter starts
 * at the true attitude instead of converging to it. Yaw starts at zero. Falls
 * back to a level start if the sample is not a clean 1 g. */
void Attitude_AlignToAccel(const IMU_Data_t *imu);

void Attitude_Update(const IMU_Data_t *imu, float dt_s);
void Attitude_GetData(Attitude_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __ATTITUDE_H__ */
