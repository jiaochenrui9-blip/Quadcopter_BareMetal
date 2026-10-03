//
// Created by game on 2026/8/30.
//
#include "flight_control.h"

#include <string.h>

static Quadcopter_FlightControl_t PitchPID;
static Quadcopter_FlightControl_t RollPID;
/* Yaw has no absolute reference on a six-axis IMU, so it is a rate loop only:
 * a single PID with no attitude loop above it. */
static PID_t YawPID;
static Quadcopter_PWMOutput_t pwm_output;
static uint32_t outer_loop_tick;
static float roll_zero;
static float pitch_zero;

/* The rate loop runs on every control tick. The attitude loop runs once every
 * QUADCOPTER_OUTER_LOOP_DIVIDER ticks; its output is held as the rate setpoint
 * until the next attitude update. */
#define QUADCOPTER_INNER_LOOP_DT_S     0.001f
#define QUADCOPTER_OUTER_LOOP_DIVIDER  4U
#define QUADCOPTER_OUTER_LOOP_DT_S     (QUADCOPTER_INNER_LOOP_DT_S * \
                                        (float)QUADCOPTER_OUTER_LOOP_DIVIDER)

/* Rate-loop integral. A proportional-only rate loop leaves a steady error
 * whenever the motors are not identical: the airframe settles tilted and the
 * yaw drifts at a constant rate. The integral supplies the constant torque
 * trim. Units: counts per (deg/s * s). The limit caps the trim itself, in
 * counts, at about 3 % duty, so a wound-up integrator cannot dominate. */
#define QUADCOPTER_RATE_KI_DEFAULT     5.0f
#define QUADCOPTER_RATE_I_LIMIT        100.0f

static void Quadcopter_AttitudeLoop_Update(const Attitude_Data_t *attitude);
static void Quadcopter_RateLoop_Update(const IMU_Data_t *imu);
static void Quadcopter_ResetAxis(Quadcopter_FlightControl_t *axis);
static void Quadcopter_PWMOutput_Clear(void);
static void Quadcopter_ClearIntegrators(void);

void Quadcopter_FlightControl_Init(void)
{
    roll_zero = 0.0f;
    pitch_zero = 0.0f;
    PID_Init(&PitchPID.Attitude_PID,15.0f,0.0f,0.0f,
        -QUADCOPTER_RATE_SETPOINT_LIMIT,QUADCOPTER_RATE_SETPOINT_LIMIT,-1000,1000);
    PID_Init(&RollPID.Attitude_PID,15.0f,0.0f,0.0f,
        -QUADCOPTER_RATE_SETPOINT_LIMIT,QUADCOPTER_RATE_SETPOINT_LIMIT,-1000,1000);
    PID_Init(&PitchPID.Omega_PID,3.0f,QUADCOPTER_RATE_KI_DEFAULT,0.0f,
        -QUADCOPTER_PID_OUTPUT_LIMIT,QUADCOPTER_PID_OUTPUT_LIMIT,
        -QUADCOPTER_RATE_I_LIMIT,QUADCOPTER_RATE_I_LIMIT);
    PID_Init(&RollPID.Omega_PID,3.0f,QUADCOPTER_RATE_KI_DEFAULT,0.0f,
        -QUADCOPTER_PID_OUTPUT_LIMIT,QUADCOPTER_PID_OUTPUT_LIMIT,
        -QUADCOPTER_RATE_I_LIMIT,QUADCOPTER_RATE_I_LIMIT);
    PID_Init(&YawPID,3.0f,QUADCOPTER_RATE_KI_DEFAULT,0.0f,
        -QUADCOPTER_PID_OUTPUT_LIMIT,QUADCOPTER_PID_OUTPUT_LIMIT,
        -QUADCOPTER_RATE_I_LIMIT,QUADCOPTER_RATE_I_LIMIT);
    Quadcopter_FlightControl_Inhibit();
}

void Quadcopter_FlightControl_SetZero(float roll_deg, float pitch_deg)
{
    roll_zero = roll_deg;
    pitch_zero = pitch_deg;
}

void Quadcopter_FlightControl_SetCommand(const RemoteControl_t *command)
{
    if (command == NULL)
    {
        return;
    }

    PID_SetTarget(&RollPID.Attitude_PID,
                  roll_zero + ((float)command->roll / QUADCOPTER_STICK_FULL_SCALE) *
                  QUADCOPTER_MAX_TILT_DEG);
    PID_SetTarget(&PitchPID.Attitude_PID,
                  pitch_zero + ((float)command->pitch / QUADCOPTER_STICK_FULL_SCALE) *
                  QUADCOPTER_MAX_TILT_DEG);
    /* Straight to the rate loop: yaw has no attitude loop to go through. */
    PID_SetTarget(&YawPID,
                  ((float)command->yaw / QUADCOPTER_STICK_FULL_SCALE) *
                  QUADCOPTER_MAX_YAW_RATE_DPS);

    pwm_output.throttle_pwm = (float)command->throttle;
}

void Quadcopter_FlightControl_SetGain(uint8_t param_id, float value)
{
    const uint8_t loop = (uint8_t)(param_id >> 4U);
    const uint8_t gain = (uint8_t)(param_id & 0x0FU);
    PID_t *pid;
    float *target;

    if ((param_id == REMOTE_PARAM_NONE) ||
        (loop >= QUADCOPTER_LOOP_COUNT) || (gain >= QUADCOPTER_GAIN_COUNT))
    {
        return;
    }

    /* Rejects NaN too: every comparison against NaN is false, so the negated
     * in-range test catches it. A NaN gain would poison the whole loop. */
    if (!((value >= QUADCOPTER_GAIN_MIN) && (value <= QUADCOPTER_GAIN_MAX)))
    {
        return;
    }

    switch (loop)
    {
        case QUADCOPTER_LOOP_ROLL_ATTITUDE:  pid = &RollPID.Attitude_PID;  break;
        case QUADCOPTER_LOOP_ROLL_RATE:      pid = &RollPID.Omega_PID;     break;
        case QUADCOPTER_LOOP_PITCH_ATTITUDE: pid = &PitchPID.Attitude_PID; break;
        case QUADCOPTER_LOOP_PITCH_RATE:     pid = &PitchPID.Omega_PID;    break;
        default:                             pid = &YawPID;                break;
    }

    switch (gain)
    {
        case QUADCOPTER_GAIN_KP: target = &pid->Kp; break;
        case QUADCOPTER_GAIN_KI: target = &pid->Ki; break;
        default:                 target = &pid->Kd; break;
    }

    if (*target == value)
    {
        /* The transmitter repeats the slot every frame; only act on a change. */
        return;
    }

    *target = value;

    /* Whatever the integrator holds was accumulated under the old gain, and for
     * Ki it is scaled by it, so carrying it across produces a step in the
     * output. Dropping it costs a little settling and avoids the kick. */
    pid->integral = 0.0f;
}

void Quadcopter_FlightControl_Update(const Attitude_Data_t *attitude,
                                     const IMU_Data_t *imu)
{
    if ((attitude == NULL) || (imu == NULL))
    {
        return;
    }

    /* The mixer holds the motors off at idle throttle, so the loops cannot act
     * on the airframe and an integrator would only wind up against the ground.
     * Keep it empty until the aircraft is actually being flown. */
    if (pwm_output.throttle_pwm <= (float)REMOTE_ARM_THROTTLE_MAX)
    {
        Quadcopter_ClearIntegrators();
    }

    if (outer_loop_tick == 0U)
    {
        Quadcopter_AttitudeLoop_Update(attitude);
    }
    outer_loop_tick++;
    if (outer_loop_tick >= QUADCOPTER_OUTER_LOOP_DIVIDER)
    {
        outer_loop_tick = 0U;
    }

    Quadcopter_RateLoop_Update(imu);
}

/* Outer loop: angle error to angular rate setpoint. */
static void Quadcopter_AttitudeLoop_Update(const Attitude_Data_t *attitude)
{
    PID_SetTarget(&PitchPID.Omega_PID,
                  PID_Calculate(&PitchPID.Attitude_PID, attitude->pitch_deg,
                                QUADCOPTER_OUTER_LOOP_DT_S));
    PID_SetTarget(&RollPID.Omega_PID,
                  PID_Calculate(&RollPID.Attitude_PID, attitude->roll_deg,
                                QUADCOPTER_OUTER_LOOP_DT_S));
}

/* Inner loop: angular rate error to motor mixer input. */
static void Quadcopter_RateLoop_Update(const IMU_Data_t *imu)
{
    pwm_output.pitch_pwm = PID_Calculate(&PitchPID.Omega_PID, imu->gyro_dps[1],
                                         QUADCOPTER_INNER_LOOP_DT_S);
    pwm_output.roll_pwm = PID_Calculate(&RollPID.Omega_PID, imu->gyro_dps[0],
                                        QUADCOPTER_INNER_LOOP_DT_S);
    pwm_output.yaw_pwm = PID_Calculate(&YawPID, imu->gyro_dps[2],
                                       QUADCOPTER_INNER_LOOP_DT_S);
}

static void Quadcopter_ClearIntegrators(void)
{
    PitchPID.Attitude_PID.integral = 0.0f;
    RollPID.Attitude_PID.integral = 0.0f;
    PitchPID.Omega_PID.integral = 0.0f;
    RollPID.Omega_PID.integral = 0.0f;
    YawPID.integral = 0.0f;
}

static void Quadcopter_ResetAxis(Quadcopter_FlightControl_t *axis)
{
    PID_Reset(&axis->Attitude_PID);
    PID_Reset(&axis->Omega_PID);
    PID_SetTarget(&axis->Attitude_PID, 0.0f);
    PID_SetTarget(&axis->Omega_PID, 0.0f);
}

/* Everything the loops carry across ticks, back to a standstill. PID_Reset also
 * clears the initialized flag, so re-enabling re-seeds the derivative history
 * instead of differentiating across the gap. */
void Quadcopter_FlightControl_Inhibit(void)
{
    Quadcopter_ResetAxis(&PitchPID);
    Quadcopter_ResetAxis(&RollPID);
    PID_Reset(&YawPID);
    PID_SetTarget(&YawPID, 0.0f);

    /* Zero means the attitude loop runs first on the tick that re-enables, so
     * the rate loop never uses a setpoint from before the outage. */
    outer_loop_tick = 0U;
    Quadcopter_PWMOutput_Clear();
}

/* Zeroes every mixer input, throttle included. */
static void Quadcopter_PWMOutput_Clear(void)
{
    memset(&pwm_output, 0, sizeof(pwm_output));
}

void Quadcopter_FlightControl_GetPWMOutput(Quadcopter_PWMOutput_t *output)
{
    if (output != NULL)
    {
        *output = pwm_output;
    }
}
