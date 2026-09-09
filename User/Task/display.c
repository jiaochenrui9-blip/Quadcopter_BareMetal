/**
  ******************************************************************************
  * @file    display.c
  * @brief   Status page on the I2C1 DMA OLED.
  *
  * Called at 10 Hz by the bare-metal main loop. Data producers also run in
  * main; OLED_Update snapshots the page and OLED_Poll advances the DMA chain.
  ******************************************************************************
  */

#include "display.h"

#include "OLED.h"
#include "flight_control.h"
#include "imu_task.h"
#include "remote_task.h"

void Display_Init(void)
{
  OLED_Init();
  OLED_Clear();
  OLED_ShowString(0, 24, "QUADCOPTER", OLED_8X16);
  OLED_Update();
}

void Display_Update(void)
{
  RemoteControl_t remote;

  RemoteTask_GetControl(&remote);
  OLED_Clear();

  /* Attitude, large enough to read while holding the airframe. */
  OLED_ShowString(0, 0, "R", OLED_8X16);
  OLED_ShowFloatNum(8, 0, g_attitude_data.roll_deg, 3, 1, OLED_8X16);
  OLED_ShowString(64, 0, "P", OLED_8X16);
  OLED_ShowFloatNum(72, 0, g_attitude_data.pitch_deg, 3, 1, OLED_8X16);

  OLED_ShowString(0, 16, "Y", OLED_8X16);
  OLED_ShowFloatNum(8, 16, g_attitude_data.yaw_deg, 3, 1, OLED_8X16);
  OLED_ShowString(64, 16, "T", OLED_8X16);
  OLED_ShowNum(72, 16, remote.throttle, 4, OLED_8X16);

  /* The three gates that decide whether the motors may run at all. */
  OLED_ShowString(0, 34, "CAL", OLED_6X8);
  OLED_ShowNum(24, 34, g_gyro_calibrated, 1, OLED_6X8);
  OLED_ShowString(42, 34, "LNK", OLED_6X8);
  OLED_ShowNum(66, 34, RemoteTask_IsLinkUp(), 1, OLED_6X8);
  OLED_ShowString(84, 34, "ARM", OLED_6X8);
  OLED_ShowNum(108, 34, RemoteTask_IsControlAllowed(), 1, OLED_6X8);

  OLED_ShowString(0, 44, "FRM", OLED_6X8);
  OLED_ShowNum(24, 44, remote.frame_count, 6, OLED_6X8);
  OLED_ShowString(78, 44, "ID", OLED_6X8);
  OLED_ShowHexNum(96, 44, remote.param_id, 2, OLED_6X8);

  OLED_ShowString(0, 54, "GY", OLED_6X8);
  OLED_ShowFloatNum(18, 54, g_imu_data.gyro_dps[1], 2, 1, OLED_6X8);
  OLED_ShowString(66, 54, "TMP", OLED_6X8);
  OLED_ShowFloatNum(90, 54, g_imu_data.temperature_deg_c, 2, 0, OLED_6X8);

  OLED_Update();
}
