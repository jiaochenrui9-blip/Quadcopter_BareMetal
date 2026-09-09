#ifndef QUADCOPTER_BAREMETAL_H
#define QUADCOPTER_BAREMETAL_H

#include "remote_task.h"

extern volatile uint8_t ICM42688_Ready;
extern volatile uint32_t g_imu_drdy_count;
extern volatile uint32_t g_imu_coalesced_count;
extern uint32_t g_imu_frame_count;
extern uint32_t g_imu_error_count;
extern volatile uint8_t Remote_Ready;
extern volatile uint8_t g_attitude_zero_calibrated;

HAL_StatusTypeDef BareMetal_Init(const NRF24_RX_Handle_t *radio);
void BareMetal_Update(void);

#endif
