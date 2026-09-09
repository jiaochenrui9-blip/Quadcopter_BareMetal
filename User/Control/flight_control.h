//
// Created by game on 2026/8/30.
//

#ifndef QUADCOPTER_FLIGHT_CONTROL_H
#define QUADCOPTER_FLIGHT_CONTROL_H

#include "PID.h"
#include "attitude.h"
#include "imu_data.h"
#include "remote_task.h"

/* Stick full scale maps onto these. Roll and pitch drive the attitude loop, so
 * they are angles; yaw has no attitude loop, so it drives the rate loop. */
#define QUADCOPTER_STICK_FULL_SCALE    1000.0f
#define QUADCOPTER_MAX_TILT_DEG        30.0f
#define QUADCOPTER_MAX_YAW_RATE_DPS    180.0f

/* The two loops speak different units. An attitude loop emits a rate setpoint
 * in degrees per second; a rate loop emits raw PWM counts for the mixer. */
#define QUADCOPTER_RATE_SETPOINT_LIMIT 200.0f
#define QUADCOPTER_PID_OUTPUT_LIMIT    300.0f

typedef struct
{
    PID_t Attitude_PID;
    PID_t Omega_PID;
} Quadcopter_FlightControl_t;

typedef struct
{
    float throttle_pwm;
    float pitch_pwm;
    float roll_pwm;
    float yaw_pwm;
} Quadcopter_PWMOutput_t;

void Quadcopter_FlightControl_Init(void);
/* Measured level attitude, degrees. Added to stick targets; feedback is unchanged.
 * Inhibit retains these offsets; Init clears them for the next startup calibration. */
void Quadcopter_FlightControl_SetZero(float roll_deg, float pitch_deg);

/* Converts one received frame into loop setpoints and base throttle. */
void Quadcopter_FlightControl_SetCommand(const RemoteControl_t *command);

/* Parameter id: high nibble picks the loop, low nibble picks the gain. So 0x00
 * is roll attitude Kp, 0x12 is roll rate Kd, 0x40 is yaw rate Kp. */
#define QUADCOPTER_LOOP_ROLL_ATTITUDE   0x0U
#define QUADCOPTER_LOOP_ROLL_RATE       0x1U
#define QUADCOPTER_LOOP_PITCH_ATTITUDE  0x2U
#define QUADCOPTER_LOOP_PITCH_RATE      0x3U
#define QUADCOPTER_LOOP_YAW_RATE        0x4U
#define QUADCOPTER_LOOP_COUNT           0x5U

#define QUADCOPTER_GAIN_KP              0x0U
#define QUADCOPTER_GAIN_KI              0x1U
#define QUADCOPTER_GAIN_KD              0x2U
#define QUADCOPTER_GAIN_COUNT           0x3U

/* A negative gain is positive feedback and diverges immediately, so the lower
 * bound is not a tuning preference. The upper bound is just a sanity ceiling. */
#define QUADCOPTER_GAIN_MIN             0.0f
#define QUADCOPTER_GAIN_MAX             100.0f

/* Applies one gain from the tuning slot of a received frame. Ignores an id of
 * REMOTE_PARAM_NONE, an unknown id, and a value that is NaN or out of range.
 * Safe to call every frame: writing the value already in place does nothing. */
void Quadcopter_FlightControl_SetGain(uint8_t param_id, float value);

/* Call once per rate-loop tick (1 kHz). The attitude loop is decimated
 * internally, so this is the only entry point the control task needs. */
void Quadcopter_FlightControl_Update(const Attitude_Data_t *attitude,
                                     const IMU_Data_t *imu);
void Quadcopter_FlightControl_GetPWMOutput(Quadcopter_PWMOutput_t *output);

/* Failsafe path: zeroes the outputs and holds every PID in reset. Call it in
 * place of _Update on ticks where control must not run. */
void Quadcopter_FlightControl_Inhibit(void);

#endif //QUADCOPTER_FLIGHT_CONTROL_H
