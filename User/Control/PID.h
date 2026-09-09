#ifndef PID_H
#define PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float target;
    float actual;
    float last_actual;

    float error0;
    float error1;

    float integral;
    float out;

    float outMax;
    float outMin;

    float integralMax;
    float integralMin;

    uint8_t initialized;
} PID_t;

void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float outMin,
              float outMax,
              float integralMin,
              float integralMax);

void PID_SetTarget(PID_t *pid, float target);
float PID_Calculate(PID_t *pid, float actual, float dt);
void PID_Reset(PID_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
