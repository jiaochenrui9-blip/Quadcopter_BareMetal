/**
  ******************************************************************************
  * @file    motor.c
  * @brief   Four motor PWM outputs on TIM2 CH1..CH4 (PA0..PA3).
  *
  * Motor index to channel: 0->CH1 (PA0), 1->CH2 (PA1), 2->CH3 (PA2),
  * 3->CH4 (PA3). Which physical motor sits on which pin is a wiring fact, so
  * the mixer owns that mapping, not this file.
  ******************************************************************************
  */

#include "motor.h"
#include "tim.h"

static const uint32_t motor_channel[MOTOR_COUNT] =
{
  TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

static uint32_t Motor_ToCompare(float duty);

HAL_StatusTypeDef Motor_Init(void)
{
  uint32_t index;

  Motor_StopAll();

  for (index = 0U; index < MOTOR_COUNT; index++)
  {
    if (HAL_TIM_PWM_Start(&htim2, motor_channel[index]) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

void Motor_Apply(const Motor_Output_t *output)
{
  uint32_t index;

  if (output == NULL)
  {
    return;
  }

  for (index = 0U; index < MOTOR_COUNT; index++)
  {
    __HAL_TIM_SET_COMPARE(&htim2, motor_channel[index],
                          Motor_ToCompare(output->duty[index]));
  }
}

void Motor_StopAll(void)
{
  uint32_t index;

  for (index = 0U; index < MOTOR_COUNT; index++)
  {
    __HAL_TIM_SET_COMPARE(&htim2, motor_channel[index], 0U);
  }
}

/* The mixer already clamps, but a compare value above the reload would wrap to
 * a near-zero duty rather than saturating, so this stays as a hard backstop. */
static uint32_t Motor_ToCompare(float duty)
{
  if (!(duty > 0.0f))          /* also catches NaN */
  {
    return 0U;
  }
  if (duty >= MOTOR_PWM_MAX)
  {
    return (uint32_t)MOTOR_PWM_MAX;
  }
  return (uint32_t)duty;
}
