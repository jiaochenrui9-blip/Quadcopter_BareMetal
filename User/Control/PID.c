#include "PID.h"

#include <stddef.h>

void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float outMin,
              float outMax,
              float integralMin,
              float integralMax)
{
    pid->initialized = 0U;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->last_actual = 0.0f;
    pid->error0 = 0.0f;
    pid->error1 = 0.0f;
    pid->integral = 0.0f;
    pid->out = 0.0f;

    pid->outMin = outMin;
    pid->outMax = outMax;
    pid->integralMin = integralMin;
    pid->integralMax = integralMax;
}

void PID_SetTarget(PID_t *pid, float target)
{
    pid->target = target;
}

float PID_Calculate(PID_t *pid, float actual, float dt)
{
    float p_term;
    float d_term;
    float integral_candidate;
    float i_candidate;
    float output_candidate;
    float i_term;

    if ((pid == NULL) || (dt <= 0.0f))
    {
        return 0.0f;
    }

    if (pid->initialized == 0U)
    {
        pid->last_actual = actual;
        pid->error0 = pid->target - actual;
        pid->error1 = pid->error0;
        pid->initialized = 1U;
    }

    pid->actual = actual;
    pid->error1 = pid->error0;
    pid->error0 = pid->target - pid->actual;

    p_term = pid->Kp * pid->error0;
    d_term = -pid->Kd * (pid->actual - pid->last_actual) / dt;

    integral_candidate = pid->integral + 0.5f * (pid->error0 + pid->error1) * dt;
    i_candidate = pid->Ki * integral_candidate;

    if (i_candidate > pid->integralMax)
    {
        i_candidate = pid->integralMax;
        if (pid->Ki != 0.0f)
        {
            integral_candidate = i_candidate / pid->Ki;
        }
    }
    else if (i_candidate < pid->integralMin)
    {
        i_candidate = pid->integralMin;
        if (pid->Ki != 0.0f)
        {
            integral_candidate = i_candidate / pid->Ki;
        }
    }

    output_candidate = p_term + i_candidate + d_term;
    if (((output_candidate <= pid->outMax) && (output_candidate >= pid->outMin)) ||
        ((output_candidate > pid->outMax) && (pid->error0 < 0.0f)) ||
        ((output_candidate < pid->outMin) && (pid->error0 > 0.0f)))
    {
        pid->integral = integral_candidate;
    }

    i_term = pid->Ki * pid->integral;
    pid->out = p_term + i_term + d_term;

    if (pid->out > pid->outMax)
    {
        pid->out = pid->outMax;
    }
    else if (pid->out < pid->outMin)
    {
        pid->out = pid->outMin;
    }

    pid->last_actual = pid->actual;
    return pid->out;
}

void PID_Reset(PID_t *pid)
{
    pid->initialized = 0U;
    pid->actual = 0.0f;
    pid->last_actual = 0.0f;
    pid->error0 = 0.0f;
    pid->error1 = 0.0f;
    pid->integral = 0.0f;
    pid->out = 0.0f;
}
