/* Actual application, MPU6050, radio parser, attitude and control sources are
 * compiled below the HAL boundary. This is a software sequence test, not an
 * electrical timing test, motor model or flight simulation. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "baremetal.h"
#include "imu_task.h"
#include "motor.h"

I2C_HandleTypeDef hi2c2;
GPIO_TypeDef test_gpio;
static uint32_t tick, primask, dma_starts, motor_applies, motor_stops;
static uint8_t *dma_buffer;
static uint8_t radio_packet[20], packet_ready, sample_interrupt_enabled;
static uint8_t notify_during_motor;
static Motor_Output_t last_motor;
void HAL_GPIO_EXTI_Callback(uint16_t pin);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *bus);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *bus);
static void finish_sample(void);

uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1; }
void __set_PRIMASK(uint32_t value) { primask = value; }
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { assert(!primask); tick += ms; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{ (void)port; (void)pin; return GPIO_PIN_SET; }

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)reg_size; (void)timeout;
    assert(!primask && bus == &hi2c2 && address == 0xD0 && reg == 0x75 && size == 1);
    *data = 0x68;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)reg_size; (void)timeout;
    assert(!primask && bus == &hi2c2 && address == 0xD0 && size == 1);
    if (reg == 0x38) sample_interrupt_enabled = *data;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *bus, uint16_t address,
    uint16_t reg, uint16_t reg_size, uint8_t *data, uint16_t size)
{
    (void)reg_size;
    assert(!primask && !dma_buffer && bus == &hi2c2 && address == 0xD0 && reg == 0x3B && size == 14);
    dma_buffer = data;
    dma_starts++;
    return HAL_OK;
}
HAL_StatusTypeDef NRF24_RX_Init(NRF24_RX_Handle_t *radio)
{ (void)radio; return HAL_OK; }
HAL_StatusTypeDef NRF24_RX_IsDataReady(NRF24_RX_Handle_t *radio, uint8_t *ready)
{ (void)radio; *ready = packet_ready; return HAL_OK; }
HAL_StatusTypeDef NRF24_RX_Read(NRF24_RX_Handle_t *radio, uint8_t *data)
{ (void)radio; assert(!primask && packet_ready); memcpy(data, radio_packet, 20); packet_ready = 0; return HAL_OK; }
HAL_StatusTypeDef NRF24_RX_ClearInterrupts(NRF24_RX_Handle_t *radio)
{ (void)radio; packet_ready = 0; return HAL_OK; }
HAL_StatusTypeDef Motor_Init(void) { Motor_StopAll(); return HAL_OK; }
void Motor_StopAll(void) { memset(&last_motor, 0, sizeof(last_motor)); motor_stops++; }
void Motor_Apply(const Motor_Output_t *output)
{
    assert(!primask && g_gyro_calibrated);
    last_motor = *output;
    motor_applies++;
    for (unsigned i=0; i<4; i++) assert(isfinite(output->duty[i]) && output->duty[i]>=0 && output->duty[i]<=3599);
    if (notify_during_motor) {
        notify_during_motor = 0;
        HAL_GPIO_EXTI_Callback(MPU6050_INT_PIN);
        finish_sample();
    }
}
static void receive_radio(uint16_t throttle)
{
    memset(radio_packet, 0, sizeof(radio_packet));
    radio_packet[6] = (uint8_t)throttle;
    radio_packet[7] = (uint8_t)(throttle >> 8);
    radio_packet[8] = 0xFF;
    packet_ready = 1;
    HAL_GPIO_EXTI_Callback(REMOTE_NRF24_IRQ_PIN);
    BareMetal_Update();
    assert(!packet_ready && !Remote_Ready);
}
static void finish_sample(void)
{
    /* Constant gyro bias: +2, -4, +6 degrees/second before calibration. */
    const int16_t raw[7] = {426,116,3816,0,131,-262,393};
    assert(dma_buffer);
    for (unsigned i=0; i<7; i++) {
        dma_buffer[2*i] = (uint8_t)((uint16_t)raw[i] >> 8);
        dma_buffer[2*i+1] = (uint8_t)raw[i];
    }
    dma_buffer = NULL;
    HAL_I2C_MemRxCpltCallback(&hi2c2);
    tick++;
}
static void complete_sample(void)
{
    finish_sample();
    BareMetal_Update();
}
static void start_sample(void)
{
    unsigned before = dma_starts;
    HAL_GPIO_EXTI_Callback(MPU6050_INT_PIN);
    assert(dma_buffer && dma_starts == before + 1);
}
int main(void)
{
    NRF24_RX_Handle_t radio = {0};
    assert(BareMetal_Init(&radio) == HAL_OK);
    assert(!g_gyro_calibrated && sample_interrupt_enabled == 1 && !dma_buffer);
    BareMetal_Update();
    receive_radio(0); /* Keep the existing idle-throttle arming rule. */
    for (unsigned frame=1; frame<=1000; frame++) {
        if (frame%100 == 0) receive_radio(0);
        start_sample();
        assert(g_gyro_calibrated == 0); /* Starting DMA must never count a sample. */
        BareMetal_Update();
        assert(g_gyro_calibrated == 0);
        complete_sample();
        assert(g_gyro_calibrated == (frame == 1000));
        assert(motor_applies == 0);
    }
    for (unsigned axis=0; axis<3; axis++) assert(fabsf(g_imu_data.gyro_dps[axis]) < 0.0001f);
    puts("PASS: unchanged MPU driver calibrates after exactly 1000 completed frames; constant bias is removed; no control before calibration");

    for (unsigned frame=1; frame<=1000; frame++) {
        if (frame%100 == 0) receive_radio(0);
        /* Iterations without a completed frame must not advance calibration. */
        for (unsigned idle=0; idle<4; idle++) BareMetal_Update();
        assert(!g_attitude_zero_calibrated);
        start_sample();
        assert(!g_attitude_zero_calibrated);
        complete_sample();
        assert(g_attitude_zero_calibrated == (frame == 1000));
        assert(motor_applies == 0);
    }
    puts("PASS: level-zero calibration waits 500 fresh frames and averages 500 more; no motor control until complete");

    receive_radio(500);
    start_sample();
    assert(motor_applies == 0);
    complete_sample();
    assert(motor_applies == 1 && last_motor.duty[0] > 0);
    BareMetal_Update();
    assert(motor_applies == 1);
    puts("PASS: one completed DMA frame produces one control/mixer/motor update; no stale-frame control");

    start_sample();
    notify_during_motor = 1;
    complete_sample();
    assert(!dma_buffer && MPU6050_DmaReady && motor_applies == 2);
    unsigned before = dma_starts;
    /* Even after transfer completion the raw buffer is owned until parsing. */
    HAL_GPIO_EXTI_Callback(MPU6050_INT_PIN);
    assert(dma_starts == before && !dma_buffer && !MPU6050_Error);
    BareMetal_Update();
    assert(!MPU6050_DmaReady && motor_applies == 3);
    start_sample();
    HAL_GPIO_EXTI_Callback(MPU6050_INT_PIN);
    assert(dma_buffer && dma_starts == before+1 && !MPU6050_Error);
    puts("PASS: EXTI starts DMA directly; completion during control survives; busy/unparsed frames cannot be overwritten");

    dma_buffer = NULL;
    HAL_I2C_ErrorCallback(&hi2c2);
    BareMetal_Update();
    for (unsigned i=0;i<4;i++) assert(last_motor.duty[i] == 0);
    start_sample(); complete_sample();
    assert(last_motor.duty[0] > 0);
    tick += 201;
    before = motor_applies;
    unsigned stops_before = motor_stops;
    BareMetal_Update();
    assert(motor_applies == before && motor_stops > stops_before);
    for (unsigned i=0;i<4;i++) assert(last_motor.duty[i] == 0);
    receive_radio(500);
    start_sample(); complete_sample();
    assert(motor_applies == before); /* Reconnect at high throttle stays inhibited. */
    receive_radio(0); receive_radio(500);
    start_sample(); complete_sample();
    assert(motor_applies == before+1);
    puts("PASS: I2C error stops output; lost radio stops without an IMU event; reconnection requires idle throttle again");
    assert(!primask);
    return 0;
}
