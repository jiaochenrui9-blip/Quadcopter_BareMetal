/**
  ******************************************************************************
  * @file    mixer.c
  * @brief   Throttle plus roll/pitch/yaw corrections to four motor duties.
  *
  * X layout, seen from above, nose up the page. CW/CCW is the propeller's own
  * rotation, also seen from above; an airframe yaws opposite to its props, so
  * the CW pair is what produces a positive (nose-left) yaw.
  *
  *      M1 (CW)        M2 (CCW)        M1 -> TIM2_CH1 (PA0)   front left
  *          \           /              M2 -> TIM2_CH2 (PA1)   front right
  *           \         /               M3 -> TIM2_CH3 (PA2)   rear right
  *            [ nose ^ ]               M4 -> TIM2_CH4 (PA3)   rear left
  *           /         \
  *          /           \
  *      M4 (CCW)       M3 (CW)
  *
  * Body frame follows the attitude filter: x forward, y left, z up. So a
  * positive roll drops the right side, a positive pitch raises the nose, and a
  * positive yaw turns the nose left.
  ******************************************************************************
  */

#include "mixer.h"

/* Sign of each correction per motor, in motor index order M1..M4. Flip a
 * column here if a bench test shows an axis fighting instead of correcting. */
static const float mixer_roll_sign[MOTOR_COUNT]  = { +1.0f, -1.0f, -1.0f, +1.0f };
static const float mixer_pitch_sign[MOTOR_COUNT] = { +1.0f, +1.0f, -1.0f, -1.0f };
static const float mixer_yaw_sign[MOTOR_COUNT]   = { +1.0f, -1.0f, +1.0f, -1.0f };

static float Mixer_Clamp(float value, float min, float max);

void Mixer_Update(const Quadcopter_PWMOutput_t *command, Motor_Output_t *output)
{
  float correction[MOTOR_COUNT];
  float lowest;
  float highest;
  float base;
  uint32_t index;

  if ((command == NULL) || (output == NULL))
  {
    return;
  }

  /* Armed but stick down: hold the props still rather than idle them. Below
   * this the loops have no authority worth spending anyway. */
  if (command->throttle_pwm <= (float)REMOTE_ARM_THROTTLE_MAX)
  {
    for (index = 0U; index < MOTOR_COUNT; index++)
    {
      output->duty[index] = 0.0f;
    }
    return;
  }

  /* Corrections first, with no throttle in them. The differences between these
   * four numbers are what actually rotate the airframe, so they are the part
   * worth protecting when something has to give. */
  for (index = 0U; index < MOTOR_COUNT; index++)
  {
    correction[index] = (mixer_roll_sign[index] * command->roll_pwm) +
                        (mixer_pitch_sign[index] * command->pitch_pwm) +
                        (mixer_yaw_sign[index] * command->yaw_pwm);
  }

  lowest = correction[0];
  highest = correction[0];
  for (index = 1U; index < MOTOR_COUNT; index++)
  {
    if (correction[index] < lowest)
    {
      lowest = correction[index];
    }
    if (correction[index] > highest)
    {
      highest = correction[index];
    }
  }

  /* If the spread on its own is wider than the output range, no amount of
   * shifting fits it and the corrections have to shrink together. Scaling all
   * three axes by the same factor keeps their ratio, so the aircraft still
   * rotates the way the loops asked, just less hard. */
  if ((highest - lowest) > MOTOR_PWM_MAX)
  {
    const float scale = MOTOR_PWM_MAX / (highest - lowest);

    for (index = 0U; index < MOTOR_COUNT; index++)
    {
      correction[index] *= scale;
    }
    lowest *= scale;
    highest *= scale;
  }

  base = (command->throttle_pwm / QUADCOPTER_STICK_FULL_SCALE) * MOTOR_PWM_MAX;

  /* Shift the whole set rather than clip individual motors. Clipping one motor
   * silently changes the differential the loops asked for, and it does that
   * exactly when authority matters most, at low throttle. Moving all four
   * together spends throttle instead, which is the cheaper thing to lose. */
  base = Mixer_Clamp(base, -lowest, MOTOR_PWM_MAX - highest);

  for (index = 0U; index < MOTOR_COUNT; index++)
  {
    output->duty[index] = base + correction[index];
  }
}

static float Mixer_Clamp(float value, float min, float max)
{
  if (value < min)
  {
    return min;
  }
  if (value > max)
  {
    return max;
  }
  return value;
}
