/* Closed-loop sign and stability check of the real estimator, flight control
 * and mixer against a small rigid-body model.
 *
 * Why this exists: the other host tests replace the plant with stubs, so they
 * confirm the data flow but cannot notice a control loop with the wrong sign.
 * Here the model is the physical truth (right-handed body frame, x forward,
 * y left, z up; thrust above the centre of mass lifts that side), and the
 * assertions are about what a stable aircraft must do, not about numbers the
 * code happens to produce.
 *
 * The inertia and thrust numbers describe a small brushed quad and are only
 * representative. The checks are about sign and convergence, not tuning.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "attitude.h"
#include "flight_control.h"
#include "mixer.h"

#define PI_D 3.14159265358979323846
#define ICM42688_SAMPLE_PERIOD_S_TEST 0.001F /* = ICM42688_SAMPLE_PERIOD_S */
#define PLANT_DT 0.0005
#define CONTROL_EVERY 2 /* plant steps per 1 kHz control tick */

typedef struct
{
    double q[4];       /* body -> world */
    double w[3];       /* body rates, rad/s */
    double thrust[4];  /* normalised motor output 0..1 */
    double efficiency[4];
} Plant;

static const double arm_m = 0.04;
static const double inertia = 2.0e-5;
static const double thrust_max_n = 0.12;
static const double motor_tau_s = 0.02;
static const double yaw_per_thrust = 0.01;
static const double damping = 1.0e-6;
/* Motor index order M1 FL, M2 FR, M3 RR, M4 RL; spin CW = +1. */
static const double pos_x[4] = { +1, +1, -1, -1 };
static const double pos_y[4] = { +1, -1, -1, +1 };
static const double spin[4] = { +1, -1, +1, -1 };

static void qmul(const double *a, const double *b, double *r)
{
    r[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    r[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    r[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    r[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

/* World "up" expressed in the body frame, which is what an accelerometer reads. */
static void up_in_body(const double *q, double *v)
{
    v[0] = 2 * (q[1] * q[3] - q[0] * q[2]);
    v[1] = 2 * (q[0] * q[1] + q[2] * q[3]);
    v[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

static double true_roll_deg(const Plant *p)
{
    double u[3];
    up_in_body(p->q, u);
    return atan2(u[1], u[2]) * 180.0 / PI_D;
}

static double true_pitch_deg(const Plant *p)
{
    double u[3];
    up_in_body(p->q, u);
    return asin(-u[0]) * 180.0 / PI_D;
}

static void plant_init(Plant *p, double roll_deg, double pitch_deg, double yaw_rate_dps)
{
    const double hr = roll_deg * PI_D / 360.0;
    const double hp = pitch_deg * PI_D / 360.0;
    /* Roll about +x and pitch about +y, both right-handed. */
    const double qx[4] = { cos(hr), sin(hr), 0, 0 };
    const double qy[4] = { cos(hp), 0, sin(hp), 0 };

    memset(p, 0, sizeof(*p));
    qmul(qx, qy, p->q);
    p->w[2] = yaw_rate_dps * PI_D / 180.0;
    for (int i = 0; i < 4; ++i) p->efficiency[i] = 1.0;
}

static void plant_step(Plant *p, const Motor_Output_t *motor)
{
    double force[4];
    double torque[3] = { 0, 0, 0 };
    double dq[4];
    double qd[4];
    double norm = 0;

    for (int i = 0; i < 4; ++i)
    {
        const double target = motor->duty[i] / MOTOR_PWM_MAX;
        p->thrust[i] += (target - p->thrust[i]) * PLANT_DT / motor_tau_s;
        force[i] = thrust_max_n * 0.5 * (p->thrust[i] * p->thrust[i] + p->thrust[i]) *
                   p->efficiency[i];
        /* r x F for a force along +z at (x, y): (y F, -x F, 0). */
        torque[0] += pos_y[i] * arm_m * force[i];
        torque[1] += -pos_x[i] * arm_m * force[i];
        torque[2] += spin[i] * yaw_per_thrust * force[i];
    }
    for (int i = 0; i < 3; ++i) p->w[i] += (torque[i] - damping * p->w[i]) / inertia * PLANT_DT;

    dq[0] = 0;
    dq[1] = p->w[0];
    dq[2] = p->w[1];
    dq[3] = p->w[2];
    qmul(p->q, dq, qd);
    for (int i = 0; i < 4; ++i)
    {
        p->q[i] += 0.5 * qd[i] * PLANT_DT;
        norm += p->q[i] * p->q[i];
    }
    norm = sqrt(norm);
    for (int i = 0; i < 4; ++i) p->q[i] /= norm;
}

typedef struct
{
    Plant plant;
    Motor_Output_t motor;
    RemoteControl_t command;
    unsigned step;
} Loop;

static void loop_init(Loop *l, double roll_deg, double pitch_deg, double yaw_rate_dps,
                      unsigned throttle)
{
    IMU_Data_t imu;
    double up[3];

    memset(l, 0, sizeof(*l));
    plant_init(&l->plant, roll_deg, pitch_deg, yaw_rate_dps);
    l->command.throttle = (uint16_t)throttle;
    l->command.param_id = REMOTE_PARAM_NONE;

    up_in_body(l->plant.q, up);
    for (int i = 0; i < 3; ++i)
    {
        imu.acc_g[i] = (float)up[i];
        imu.gyro_dps[i] = 0.0f;
    }
    imu.temperature_deg_c = 25.0f;
    Attitude_Init();
    Attitude_AlignToAccel(&imu);
    Quadcopter_FlightControl_Init();
    Quadcopter_FlightControl_SetZero(0.0f, 0.0f);
}

/* One plant step; every CONTROL_EVERY-th also runs the controller at 1 kHz,
 * in the same order BareMetal_ProcessFrame uses. */
static void loop_step(Loop *l)
{
    if ((l->step % CONTROL_EVERY) == 0U)
    {
        IMU_Data_t imu;
        Attitude_Data_t attitude;
        Quadcopter_PWMOutput_t pwm;
        double up[3];

        up_in_body(l->plant.q, up);
        for (int i = 0; i < 3; ++i)
        {
            imu.acc_g[i] = (float)up[i];
            imu.gyro_dps[i] = (float)(l->plant.w[i] * 180.0 / PI_D);
        }
        imu.temperature_deg_c = 25.0f;
        Attitude_Update(&imu, ICM42688_SAMPLE_PERIOD_S_TEST);
        Attitude_GetData(&attitude);
        Quadcopter_FlightControl_SetCommand(&l->command);
        Quadcopter_FlightControl_Update(&attitude, &imu);
        Quadcopter_FlightControl_GetPWMOutput(&pwm);
        Mixer_Update(&pwm, &l->motor);
    }
    plant_step(&l->plant, &l->motor);
    ++l->step;
}

static void run_seconds(Loop *l, double seconds)
{
    const unsigned steps = (unsigned)(seconds / PLANT_DT);
    for (unsigned i = 0; i < steps; ++i) loop_step(l);
}

static double yaw_rate_dps(const Loop *l)
{
    return l->plant.w[2] * 180.0 / PI_D;
}

int main(void)
{
    Loop l;

    /* The first control tick already shows which way each correction pushes. */
    loop_init(&l, 0.0, 10.0, 0.0, 500U); /* nose down */
    loop_step(&l);
    assert(l.motor.duty[0] + l.motor.duty[1] > l.motor.duty[2] + l.motor.duty[3]);
    loop_init(&l, 10.0, 0.0, 0.0, 500U); /* left side up */
    loop_step(&l);
    assert(l.motor.duty[0] + l.motor.duty[3] < l.motor.duty[1] + l.motor.duty[2]);
    puts("PASS: first correction lifts the low side (nose-down -> front, left-up -> right)");

    /* Negative feedback on each axis, and on all of them together. */
    loop_init(&l, 0.0, 10.0, 0.0, 500U);
    run_seconds(&l, 3.0);
    assert(fabs(true_pitch_deg(&l.plant)) < 0.5 && fabs(true_roll_deg(&l.plant)) < 0.5);
    loop_init(&l, 10.0, 0.0, 0.0, 500U);
    run_seconds(&l, 3.0);
    assert(fabs(true_pitch_deg(&l.plant)) < 0.5 && fabs(true_roll_deg(&l.plant)) < 0.5);
    loop_init(&l, 0.0, 0.0, 60.0, 500U);
    run_seconds(&l, 3.0);
    assert(fabs(yaw_rate_dps(&l)) < 2.0);
    loop_init(&l, -12.0, 15.0, 30.0, 500U);
    run_seconds(&l, 4.0);
    assert(fabs(true_pitch_deg(&l.plant)) < 0.5 && fabs(true_roll_deg(&l.plant)) < 0.5);
    assert(fabs(yaw_rate_dps(&l)) < 2.0);
    puts("PASS: roll, pitch and yaw disturbances decay (closed loop is negative feedback)");

    /* A weak motor needs a standing torque trim; proportional-only loops
     * cannot supply it and settle tilted and spinning. */
    loop_init(&l, 0.0, 0.0, 0.0, 500U);
    l.plant.efficiency[0] = 0.8;
    run_seconds(&l, 8.0);
    assert(fabs(true_pitch_deg(&l.plant)) < 0.3 && fabs(true_roll_deg(&l.plant)) < 0.3);
    assert(fabs(yaw_rate_dps(&l)) < 1.0);
    puts("PASS: 20 % weaker motor is trimmed out by the rate integral");

    /* On the ground at idle throttle the mixer cuts the motors, so nothing the
     * loops do can reduce the error. The integrator must stay empty then,
     * otherwise take-off starts with a pre-loaded torque offset. */
    loop_init(&l, 20.0, -15.0, 0.0, 0U);
    for (unsigned i = 0; i < 2000U; ++i) /* 2 s held tilted, plant frozen */
    {
        IMU_Data_t imu;
        Attitude_Data_t attitude;
        Quadcopter_PWMOutput_t pwm;
        double up[3];

        up_in_body(l.plant.q, up);
        for (int k = 0; k < 3; ++k)
        {
            imu.acc_g[k] = (float)up[k];
            imu.gyro_dps[k] = 0.0f;
        }
        Attitude_Update(&imu, ICM42688_SAMPLE_PERIOD_S_TEST);
        Attitude_GetData(&attitude);
        Quadcopter_FlightControl_SetCommand(&l.command);
        Quadcopter_FlightControl_Update(&attitude, &imu);
        Quadcopter_FlightControl_GetPWMOutput(&pwm);
        Mixer_Update(&pwm, &l.motor);
        for (int k = 0; k < 4; ++k) assert(l.motor.duty[k] == 0.0f);
    }
    {
        /* Level the aircraft, lift off: the rate loop must start from zero. */
        IMU_Data_t imu = { { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, 0.0f }, 25.0f };
        Attitude_Data_t attitude;
        Quadcopter_PWMOutput_t pwm;

        Attitude_Init();
        Attitude_AlignToAccel(&imu);
        Attitude_GetData(&attitude);
        l.command.throttle = 500U;
        Quadcopter_FlightControl_SetCommand(&l.command);
        Quadcopter_FlightControl_Update(&attitude, &imu);
        Quadcopter_FlightControl_GetPWMOutput(&pwm);
        assert(fabsf(pwm.roll_pwm) < 5.0f && fabsf(pwm.pitch_pwm) < 5.0f &&
               fabsf(pwm.yaw_pwm) < 5.0f);
    }
    puts("PASS: integrators stay empty while throttle is idle");
    return 0;
}
