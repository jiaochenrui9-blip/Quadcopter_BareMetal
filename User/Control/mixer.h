/**
  ******************************************************************************
  * @file    mixer.h
  * @brief   Throttle plus roll/pitch/yaw corrections to four motor duties.
  ******************************************************************************
  */
#ifndef __MIXER_H__
#define __MIXER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "flight_control.h"
#include "motor.h"

void Mixer_Update(const Quadcopter_PWMOutput_t *command, Motor_Output_t *output);

#ifdef __cplusplus
}
#endif

#endif /* __MIXER_H__ */
