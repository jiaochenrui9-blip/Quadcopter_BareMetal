#include "baremetal.h"
#include "icm42688.h"
#include "imu_task.h"
#include "flight_control.h"
#include "mixer.h"
#include "display.h"
#include "OLED.h"

#define ATTITUDE_ZERO_AVERAGE_SAMPLES 500U
#define IMU_STALE_TIMEOUT_MS 20U
#define DISPLAY_PERIOD_MS 100U

volatile uint8_t ICM42688_Ready;
volatile uint8_t Remote_Ready;
volatile uint8_t g_gyro_calibrated;
volatile uint8_t g_attitude_zero_calibrated;
volatile uint32_t g_imu_drdy_count;
volatile uint32_t g_imu_coalesced_count;
uint32_t g_imu_frame_count;
uint32_t g_imu_error_count;
static uint32_t last_imu_ms;
static uint32_t last_display_ms;
static uint32_t attitude_zero_sample_count;
static float attitude_zero_roll_sum;
static float attitude_zero_pitch_sum;
IMU_Data_t g_imu_data;
Attitude_Data_t g_attitude_data;
RemoteControl_t g_remote_control;

static void BareMetal_Stop(void)
{
    Motor_StopAll();
    Quadcopter_FlightControl_Inhibit();
}

static void BareMetal_CalibrateAttitudeZero(void)
{
    attitude_zero_roll_sum += g_attitude_data.roll_deg;
    attitude_zero_pitch_sum += g_attitude_data.pitch_deg;
    if (++attitude_zero_sample_count >= ATTITUDE_ZERO_AVERAGE_SAMPLES)
    {
        Quadcopter_FlightControl_SetZero(
            attitude_zero_roll_sum / (float)ATTITUDE_ZERO_AVERAGE_SAMPLES,
            attitude_zero_pitch_sum / (float)ATTITUDE_ZERO_AVERAGE_SAMPLES);
        g_attitude_zero_calibrated = 1U;
    }
}

/* Clear before processing; an interrupt during work stays pending. */
static uint8_t BareMetal_TakeFlag(volatile uint8_t *flag)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t ready;
    __disable_irq();
    ready = *flag;
    *flag = 0U;
    __set_PRIMASK(primask);
    return ready;
}

HAL_StatusTypeDef BareMetal_Init(const NRF24_RX_Handle_t *radio)
{
    ICM42688_Ready = Remote_Ready = 0U;
    g_gyro_calibrated = g_attitude_zero_calibrated = 0U;
    g_imu_drdy_count = g_imu_coalesced_count = 0U;
    g_imu_frame_count = g_imu_error_count = 0U;
    attitude_zero_sample_count = 0U;
    attitude_zero_roll_sum = attitude_zero_pitch_sum = 0.0f;
    if (Motor_Init() != HAL_OK) return HAL_ERROR;
    if (ICM42688_Init() != HAL_OK) return HAL_ERROR;
    Attitude_Init();
    Quadcopter_FlightControl_Init();
    (void)RemoteTask_Init(radio);
    Remote_Ready = 1U;
    Display_Init();
    last_imu_ms = last_display_ms = HAL_GetTick();
    /* 1000 fresh gyro frames averaged inside the driver. Keep stationary. */
    return ICM42688_EnableDataReadyInterrupt();
}

static void BareMetal_ProcessFrame(void)
{
    if (!g_gyro_calibrated)
    {
        if (ICM42688_GyroCalibrationIsComplete())
        {
            g_gyro_calibrated = 1U;
            Attitude_AlignToAccel(&g_imu_data);
            Attitude_GetData(&g_attitude_data);
        }
        return;
    }
    Attitude_Update(&g_imu_data, ICM42688_SAMPLE_PERIOD_S);
    Attitude_GetData(&g_attitude_data);
    if (!g_attitude_zero_calibrated)
    {
        BareMetal_CalibrateAttitudeZero();
    }
    else if (RemoteTask_IsControlAllowed())
    {
        Quadcopter_PWMOutput_t pwm_output;
        Motor_Output_t motor_output;
        Quadcopter_FlightControl_SetCommand(&g_remote_control);
        Quadcopter_FlightControl_Update(&g_attitude_data, &g_imu_data);
        Quadcopter_FlightControl_GetPWMOutput(&pwm_output);
        Mixer_Update(&pwm_output, &motor_output);
        Motor_Apply(&motor_output);
    }
}

void BareMetal_Update(void)
{
    HAL_StatusTypeDef status;
    if (BareMetal_TakeFlag(&Remote_Ready))
    {
        RemoteTask_Update();
        RemoteTask_GetControl(&g_remote_control);
        Quadcopter_FlightControl_SetGain(g_remote_control.param_id,
                                       g_remote_control.param_value);
    }
    /* This check runs even if no new IMU frame arrives. */
    if (!RemoteTask_IsControlAllowed() || !g_gyro_calibrated ||
        !g_attitude_zero_calibrated ||
        (uint32_t)(HAL_GetTick() - last_imu_ms) >= IMU_STALE_TIMEOUT_MS)
        BareMetal_Stop();

    status = ICM42688_Consume(&g_imu_data);
    if (status == HAL_OK)
    {
        last_imu_ms = HAL_GetTick();
        ++g_imu_frame_count;
        BareMetal_ProcessFrame();
    }
    else if (status != HAL_BUSY)
    {
        ++g_imu_error_count;
        BareMetal_Stop();
    }

    if (!ICM42688_IsBusy() && BareMetal_TakeFlag(&ICM42688_Ready))
    {
        status = ICM42688_StartDmaRead();
        if (status != HAL_OK && status != HAL_BUSY)
        {
            ++g_imu_error_count;
            BareMetal_Stop();
        }
    }

    /* Display formatting is slow background work: wait until calibration ends
     * and no IMU transfer is pending. DMA panel transfers themselves overlap IMU. */
    if (g_attitude_zero_calibrated && !ICM42688_IsBusy() && !ICM42688_Ready &&
        !OLED_IsBusy() && (uint32_t)(HAL_GetTick() - last_display_ms) >= DISPLAY_PERIOD_MS)
    {
        last_display_ms = HAL_GetTick();
        Display_Update();
    }
    OLED_Poll();
}
