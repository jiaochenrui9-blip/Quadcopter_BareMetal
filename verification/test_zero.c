#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "flight_control.h"

static void check(float roll, float pitch, int16_t stick, float expected_roll, float expected_pitch)
{
    MPU6050_Data_t imu = {0};
    Attitude_Data_t attitude = {0};
    RemoteControl_t command = {0};
    Quadcopter_PWMOutput_t result;
    attitude.roll_deg = roll;
    attitude.pitch_deg = pitch;
    command.roll = command.pitch = stick;
    command.throttle = 500;
    Quadcopter_FlightControl_Inhibit();
    Quadcopter_FlightControl_SetCommand(&command);
    Quadcopter_FlightControl_Update(&attitude, &imu);
    Quadcopter_FlightControl_GetPWMOutput(&result);
    assert(fabsf(result.roll_pwm - expected_roll) < 0.001f);
    assert(fabsf(result.pitch_pwm - expected_pitch) < 0.001f);
    assert(attitude.roll_deg == roll && attitude.pitch_deg == pitch);
}
int main(void)
{
    Quadcopter_FlightControl_Init();
    Quadcopter_FlightControl_SetZero(2.0f, -1.0f);
    check(2.0f, -1.0f, 0, 0.0f, 0.0f);
    check(0.0f, 0.0f, 0, 90.0f, -45.0f);
    check(2.0f, -1.0f, 100, 135.0f, 135.0f);
    check(2.0f, -1.0f, 0, 0.0f, 0.0f);
    Quadcopter_FlightControl_Init();
    check(2.0f, -1.0f, 0, -90.0f, 45.0f);
    puts("PASS: target = zero + stick angle; measured angles unchanged; inhibit preserves zero; init clears zero");
    return 0;
}
