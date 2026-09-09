#ifndef IMU_DATA_H
#define IMU_DATA_H

/* Sensor-independent units consumed by the existing estimator/controller. */
typedef struct
{
    float acc_g[3];
    float gyro_dps[3];
    float temperature_deg_c;
} IMU_Data_t;

#endif
