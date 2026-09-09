/**
  ******************************************************************************
  * @file    attitude.c
  * @brief   Six-axis Mahony quaternion attitude update.
  ******************************************************************************
  */

#include "attitude.h"

#include <math.h>
#include <string.h>

#define ATTITUDE_RAD_PER_DEG       0.01745329251994329577F
#define ATTITUDE_DEG_PER_RAD       57.295779513082320876F
/* Correction time constant is 1/KP seconds: this is how fast the accelerometer
 * is allowed to pull the estimate around. Raising it makes start-up converge
 * faster but also lets prop vibration and translational acceleration into the
 * angle, so instead of raising it the estimate is seeded from the accelerometer
 * in Attitude_AlignToAccel and there is nothing left to converge. */
#define ATTITUDE_KP                2.0F
#define ATTITUDE_KI                0.02F
#define ATTITUDE_VECTOR_MIN_NORM   0.000001F
#define ATTITUDE_ACC_MIN_G         0.8F
#define ATTITUDE_ACC_MAX_G         1.2F
#define ATTITUDE_INTEGRAL_LIMIT    0.5F
#define ATTITUDE_ALIGN_MIN_COS     0.001F

static Attitude_Data_t attitude_data;

static float Attitude_InvSqrt(float value);
static float Attitude_Clamp(float value, float min, float max);
static void Attitude_UpdateEuler(void);

void Attitude_Init(void)
{
  memset(&attitude_data, 0, sizeof(attitude_data));
  attitude_data.q0 = 1.0F;
}

void Attitude_AlignToAccel(const IMU_Data_t *imu)
{
  float ax;
  float ay;
  float az;
  float acc_norm;
  float horizon;
  float cos_half_roll;
  float sin_half_roll;
  float cos_half_pitch;
  float sin_half_pitch;

  Attitude_Init();

  if (imu == NULL)
  {
    return;
  }

  ax = imu->acc_g[0];
  ay = imu->acc_g[1];
  az = imu->acc_g[2];
  acc_norm = sqrtf(ax * ax + ay * ay + az * az);

  /* Anything but a clean 1 g means the airframe was not at rest, and a level
   * start is a better guess than a tilt derived from a moving accelerometer. */
  if ((acc_norm <= ATTITUDE_ACC_MIN_G) || (acc_norm >= ATTITUDE_ACC_MAX_G))
  {
    return;
  }

  ax /= acc_norm;
  ay /= acc_norm;
  az /= acc_norm;

  /* At rest the body sees gravity as (-sin p, sin r cos p, cos r cos p), so the
   * length of the yz component is cos p. Going straight to half angles through
   * the sqrt identities keeps sinf and cosf out of the image entirely, which is
   * worth ~4.7 kB of flash for something that runs once. Yaw starts at zero
   * because gravity carries no heading information. */
  horizon = sqrtf(ay * ay + az * az);

  cos_half_pitch = sqrtf((1.0F + horizon) * 0.5F);
  sin_half_pitch = -ax / (2.0F * cos_half_pitch);

  if (horizon < ATTITUDE_ALIGN_MIN_COS)
  {
    /* Nose straight up or down leaves roll undefined. */
    cos_half_roll = 1.0F;
    sin_half_roll = 0.0F;
  }
  else
  {
    cos_half_roll = sqrtf((1.0F + (az / horizon)) * 0.5F);
    if (cos_half_roll < ATTITUDE_ALIGN_MIN_COS)
    {
      /* Inverted: a level start is the safer guess. */
      return;
    }
    sin_half_roll = (ay / horizon) / (2.0F * cos_half_roll);
  }

  attitude_data.q0 = cos_half_roll * cos_half_pitch;
  attitude_data.q1 = sin_half_roll * cos_half_pitch;
  attitude_data.q2 = cos_half_roll * sin_half_pitch;
  attitude_data.q3 = -sin_half_roll * sin_half_pitch;

  Attitude_UpdateEuler();
}

void Attitude_Update(const IMU_Data_t *imu, float dt_s)
{
  float q0;
  float q1;
  float q2;
  float q3;
  float gx;
  float gy;
  float gz;
  float ax;
  float ay;
  float az;
  float norm_sq;
  float acc_norm;
  float vx;
  float vy;
  float vz;
  float ex;
  float ey;
  float ez;
  float half_dt;

  if ((imu == NULL) || (dt_s <= 0.0F))
  {
    return;
  }

  q0 = attitude_data.q0;
  q1 = attitude_data.q1;
  q2 = attitude_data.q2;
  q3 = attitude_data.q3;

  gx = imu->gyro_dps[0] * ATTITUDE_RAD_PER_DEG;
  gy = imu->gyro_dps[1] * ATTITUDE_RAD_PER_DEG;
  gz = imu->gyro_dps[2] * ATTITUDE_RAD_PER_DEG;
  ax = imu->acc_g[0];
  ay = imu->acc_g[1];
  az = imu->acc_g[2];
  norm_sq = ax * ax + ay * ay + az * az;
  acc_norm = sqrtf(norm_sq);

  if ((acc_norm > ATTITUDE_ACC_MIN_G) && (acc_norm < ATTITUDE_ACC_MAX_G))
  {
    const float inv_norm = 1.0F / acc_norm;

    ax *= inv_norm;
    ay *= inv_norm;
    az *= inv_norm;

    vx = 2.0F * (q1 * q3 - q0 * q2);
    vy = 2.0F * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    attitude_data.integral_error[0] += ATTITUDE_KI * ex * dt_s;
    attitude_data.integral_error[1] += ATTITUDE_KI * ey * dt_s;
    attitude_data.integral_error[2] += ATTITUDE_KI * ez * dt_s;
    attitude_data.integral_error[0] = Attitude_Clamp(attitude_data.integral_error[0],
                                                      -ATTITUDE_INTEGRAL_LIMIT,
                                                      ATTITUDE_INTEGRAL_LIMIT);
    attitude_data.integral_error[1] = Attitude_Clamp(attitude_data.integral_error[1],
                                                      -ATTITUDE_INTEGRAL_LIMIT,
                                                      ATTITUDE_INTEGRAL_LIMIT);
    attitude_data.integral_error[2] = Attitude_Clamp(attitude_data.integral_error[2],
                                                      -ATTITUDE_INTEGRAL_LIMIT,
                                                      ATTITUDE_INTEGRAL_LIMIT);

    gx += ATTITUDE_KP * ex + attitude_data.integral_error[0];
    gy += ATTITUDE_KP * ey + attitude_data.integral_error[1];
    gz += ATTITUDE_KP * ez + attitude_data.integral_error[2];
  }

  attitude_data.gyro_corrected_dps[0] = gx * ATTITUDE_DEG_PER_RAD;
  attitude_data.gyro_corrected_dps[1] = gy * ATTITUDE_DEG_PER_RAD;
  attitude_data.gyro_corrected_dps[2] = gz * ATTITUDE_DEG_PER_RAD;

  half_dt = 0.5F * dt_s;
  gx *= half_dt;
  gy *= half_dt;
  gz *= half_dt;

  attitude_data.q0 += -q1 * gx - q2 * gy - q3 * gz;
  attitude_data.q1 += q0 * gx + q2 * gz - q3 * gy;
  attitude_data.q2 += q0 * gy - q1 * gz + q3 * gx;
  attitude_data.q3 += q0 * gz + q1 * gy - q2 * gx;

  norm_sq = attitude_data.q0 * attitude_data.q0 + attitude_data.q1 * attitude_data.q1
          + attitude_data.q2 * attitude_data.q2 + attitude_data.q3 * attitude_data.q3;
  if (norm_sq > ATTITUDE_VECTOR_MIN_NORM)
  {
    const float inv_norm = Attitude_InvSqrt(norm_sq);

    attitude_data.q0 *= inv_norm;
    attitude_data.q1 *= inv_norm;
    attitude_data.q2 *= inv_norm;
    attitude_data.q3 *= inv_norm;
  }
  else
  {
    Attitude_Init();
  }

  Attitude_UpdateEuler();
}

void Attitude_GetData(Attitude_Data_t *data)
{
  if (data != NULL)
  {
    *data = attitude_data;
  }
}

static float Attitude_InvSqrt(float value)
{
  return 1.0F / sqrtf(value);
}

static float Attitude_Clamp(float value, float min, float max)
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

static void Attitude_UpdateEuler(void)
{
  const float q0 = attitude_data.q0;
  const float q1 = attitude_data.q1;
  const float q2 = attitude_data.q2;
  const float q3 = attitude_data.q3;
  const float pitch_sin = Attitude_Clamp(2.0F * (q0 * q2 - q3 * q1), -1.0F, 1.0F);

  attitude_data.roll_deg = atan2f(2.0F * (q0 * q1 + q2 * q3),
                                  1.0F - 2.0F * (q1 * q1 + q2 * q2)) * ATTITUDE_DEG_PER_RAD;
  attitude_data.pitch_deg = asinf(pitch_sin) * ATTITUDE_DEG_PER_RAD;
  attitude_data.yaw_deg = atan2f(2.0F * (q0 * q3 + q1 * q2),
                                 1.0F - 2.0F * (q2 * q2 + q3 * q3)) * ATTITUDE_DEG_PER_RAD;
}
