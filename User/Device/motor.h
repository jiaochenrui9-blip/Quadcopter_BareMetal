/**
  ******************************************************************************
  * @file    motor.h
  * @brief   Four motor PWM outputs on TIM2 CH1..CH4 (PA0..PA3).
  ******************************************************************************
  */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define MOTOR_COUNT           4U

/* Must match the TIM2 auto-reload value set by MX_TIM2_Init. */
#define MOTOR_PWM_MAX         3599.0f

typedef struct
{
  float duty[MOTOR_COUNT];   /* 0 .. MOTOR_PWM_MAX, clamped on the way out */
} Motor_Output_t;

/* Starts the four PWM channels. Every compare register is zeroed first, so
 * nothing spins between this call and the first Motor_Apply. */
HAL_StatusTypeDef Motor_Init(void);

void Motor_Apply(const Motor_Output_t *output);
void Motor_StopAll(void);

/* Fault-path stop: writes the TIM2 compare registers directly. It needs no
 * handle, no HAL state and no clock-enabled check, so it is safe to call from
 * a fault handler or Error_Handler, where the rest of the program is suspect. */
void Motor_EmergencyStop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
