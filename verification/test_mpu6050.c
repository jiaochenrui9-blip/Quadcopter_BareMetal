/* Actual driver, baremetal scheduler, radio parser, estimator/controller and
 * OLED state machine. Hardware is mocked; this is not a timing/flight test. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "baremetal.h"
#include "mpu6050.h"
#include "imu_task.h"
#include "motor.h"
#include "OLED.h"

I2C_HandleTypeDef hi2c1, hi2c2;
GPIO_TypeDef test_gpio;
uint8_t test_i2c_busy;
static uint32_t tick, primask, dma_starts, applies, resets;
static uint8_t regs[128], *dma_rx, *oled_tx, packet[20], packet_ready;
static uint16_t oled_size;
/* Raw sensor words in register order: accel xyz, temperature, gyro xyz. The
 * accelerometer words land on (0, 0, 1 g) once the driver removes its level
 * offsets, and the temperature word on 25 degC. */
static int16_t raw[7] = {426, 117, 3816, -3920, 131, -262, 393};
static uint8_t fail_dma, fail_reg, corrupt_reg, data_ready = 1U;
static Motor_Output_t last_motor;
void HAL_GPIO_EXTI_Callback(uint16_t);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *);

uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1U; }
void __set_PRIMASK(uint32_t p) { primask = p; }
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { assert(!primask); tick += ms; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t n)
{ (void)p; (void)n; return GPIO_PIN_SET; }

/* Register file of the MPU6050 on I2C2, 7-bit address 0x68. */
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *b, uint16_t addr, uint16_t reg,
    uint16_t size, uint8_t *data, uint16_t n, uint32_t timeout)
{
    (void)timeout;
    assert(!primask && b == &hi2c2 && addr == 0xD0U && size == I2C_MEMADD_SIZE_8BIT);
    assert(n == 1U && !dma_rx);
    if (fail_reg == reg) return HAL_ERROR;
    regs[reg] = data[0];
    if (reg == 0x6BU && data[0] == 0x80U) regs[reg] = 0x40U; /* reset leaves it asleep */
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *b, uint16_t addr, uint16_t reg,
    uint16_t size, uint8_t *data, uint16_t n, uint32_t timeout)
{
    (void)timeout;
    assert(!primask && b == &hi2c2 && addr == 0xD0U && size == I2C_MEMADD_SIZE_8BIT);
    assert(n == 1U && !dma_rx);
    if (fail_reg == reg) return HAL_ERROR;
    data[0] = regs[reg] ^ (corrupt_reg == reg ? 1U : 0U);
    if (reg == 0x3AU) regs[reg] = 0U; /* INT_RD_CLEAR */
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *b, uint16_t addr, uint16_t reg,
    uint16_t size, uint8_t *data, uint16_t n)
{
    assert(!primask && b == &hi2c2 && addr == 0xD0U && size == I2C_MEMADD_SIZE_8BIT);
    assert(reg == 0x3AU && n == 15U && !dma_rx); /* INT_STATUS + 14 data bytes */
    dma_rx = data;
    ++dma_starts;
    return fail_dma ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *b, uint16_t addr,
    uint8_t *data, uint16_t n)
{
    assert(b == &hi2c1 && addr == 0x78U && !oled_tx && n > 1U);
    oled_tx = data; oled_size = n; return HAL_OK;
}
/* Both buses are re-initialised on failure: I2C1 for the OLED, I2C2 for the IMU. */
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *b)
{
    if (b == &hi2c2) { dma_rx = NULL; ++resets; }
    else { assert(b == &hi2c1); oled_tx = NULL; }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *b)
{ assert(b == &hi2c1 || b == &hi2c2); return HAL_OK; }
HAL_StatusTypeDef NRF24_RX_Init(NRF24_RX_Handle_t *r) { (void)r; return HAL_OK; }
HAL_StatusTypeDef NRF24_RX_IsDataReady(NRF24_RX_Handle_t *r, uint8_t *v)
{ (void)r; *v = packet_ready; return HAL_OK; }
HAL_StatusTypeDef NRF24_RX_Read(NRF24_RX_Handle_t *r, uint8_t *v)
{ (void)r; memcpy(v, packet, 20); packet_ready = 0U; return HAL_OK; }
HAL_StatusTypeDef NRF24_RX_ClearInterrupts(NRF24_RX_Handle_t *r)
{ (void)r; packet_ready = 0U; return HAL_OK; }
HAL_StatusTypeDef Motor_Init(void) { Motor_StopAll(); return HAL_OK; }
void Motor_StopAll(void) { memset(&last_motor, 0, sizeof(last_motor)); }
void Motor_Apply(const Motor_Output_t *p)
{
    assert(!primask && g_gyro_calibrated && g_attitude_zero_calibrated);
    last_motor = *p; ++applies;
    for (unsigned i=0; i<4; ++i) assert(isfinite(p->duty[i]) && p->duty[i]>=0 && p->duty[i]<=3599);
}
static void radio(uint16_t throttle)
{
    memset(packet, 0, sizeof(packet));
    packet[6] = (uint8_t)throttle; packet[7] = (uint8_t)(throttle >> 8);
    packet[8] = 0xFFU; packet_ready = 1U;
    HAL_GPIO_EXTI_Callback(REMOTE_NRF24_IRQ_PIN); BareMetal_Update();
}
static void request(void)
{
    uint32_t before = dma_starts;
    HAL_GPIO_EXTI_Callback(MPU6050_INT_PIN);
    assert(dma_starts == before); /* EXTI never starts I2C. */
}
static void finish(void)
{
    assert(dma_rx);
    dma_rx[0] = data_ready ? 0x01U : 0x00U; /* INT_STATUS.DATA_RDY_INT */
    for (unsigned i=0; i<7; ++i)
    {
        dma_rx[1+2*i] = (uint8_t)((uint16_t)raw[i] >> 8);
        dma_rx[2+2*i] = (uint8_t)raw[i];
    }
    dma_rx = NULL;
    HAL_I2C_MemRxCpltCallback(&hi2c2);
}
static void frame(void)
{
    ++tick; request(); BareMetal_Update(); assert(dma_rx);
    finish(); BareMetal_Update();
}
static void stopped(void)
{ for (unsigned i=0; i<4; ++i) assert(last_motor.duty[i] == 0); }

int main(void)
{
    IMU_Data_t sample;
    NRF24_RX_Handle_t cfg = {0};
    assert(MPU6050_Init() == HAL_ERROR); /* wrong WHO_AM_I */
    regs[0x75] = 0x68;
    fail_reg = 0x1B; assert(MPU6050_Init() == HAL_ERROR); fail_reg = 0;
    corrupt_reg = 0x1C; assert(MPU6050_Init() == HAL_ERROR); corrupt_reg = 0;
    assert(MPU6050_Init() == HAL_OK);
    corrupt_reg = 0x38; assert(MPU6050_EnableDataReadyInterrupt() == HAL_ERROR); corrupt_reg = 0;
    /* Units and axis order, straight from the driver while no bias is known yet:
     * a different value on every axis, so a swapped axis or a wrong scale shows. */
    raw[0] = 426 + 2048; raw[1] = 117 - 1024; /* +0.5 g, -0.25 g, 1 g on z */
    raw[4] = 12 * 655 / 10; raw[5] = -24 * 655 / 10; raw[6] = 36 * 655 / 10; /* 12, -24, 36 dps */
    assert(MPU6050_StartDmaRead() == HAL_OK); finish(); assert(MPU6050_Consume(&sample) == HAL_OK);
    assert(fabsf(sample.acc_g[0] - 0.5f) < 0.001f && fabsf(sample.acc_g[1] + 0.25f) < 0.001f);
    assert(fabsf(sample.acc_g[2] - 1.0f) < 0.001f);
    assert(fabsf(sample.gyro_dps[0] - 12.0f) < 0.001f && fabsf(sample.gyro_dps[1] + 24.0f) < 0.001f);
    assert(fabsf(sample.gyro_dps[2] - 36.0f) < 0.001f);
    raw[0] = 426; raw[1] = 117; raw[4] = 131; raw[5] = -262; raw[6] = 393;
    assert(BareMetal_Init(&cfg) == HAL_OK);
    assert(regs[0x6B]==0x01 && regs[0x19]==0x00 && regs[0x1A]==0x03);
    assert(regs[0x1B]==0x08 && regs[0x1C]==0x10);
    assert(regs[0x37]==0x10 && regs[0x38]==0x01);
    puts("PASS: identity/readback failures rejected; 1 kHz/DLPF/500 dps/8 g/DRDY configured");
    for (unsigned i=1; i<=1000; ++i)
    {
        if (i%100==1) radio(0);
        frame();
        assert(g_gyro_calibrated == (i==1000) && applies==0);
    }
    for (unsigned i=0;i<3;++i) assert(fabsf(g_imu_data.gyro_dps[i]) < 0.0001f);
    assert(fabsf(g_imu_data.acc_g[0]) < 0.001f && fabsf(g_imu_data.acc_g[1]) < 0.001f);
    assert(fabsf(g_imu_data.acc_g[2] - 1.0f) < 0.001f);
    assert(fabsf(g_imu_data.temperature_deg_c - 25.0f) < 0.01f);
    for (unsigned i=1;i<=500;++i)
    {
        if (i%100==1) radio(0);
        frame(); assert(g_attitude_zero_calibrated==(i==500) && applies==0);
    }
    radio(500); frame(); assert(applies==1 && last_motor.duty[0]>0);
    BareMetal_Update(); assert(applies==1);
    puts("PASS: 1000 DMA frames remove gyro bias; 500 attitude samples; no premature/stale control");
    request(); BareMetal_Update(); assert(MPU6050_Consume(&sample)==HAL_BUSY);
    request(); request(); assert(g_imu_coalesced_count>0);
    uint32_t count=dma_starts; BareMetal_Update(); assert(dma_starts==count);
    finish(); assert(MPU6050_StartDmaRead()==HAL_BUSY); /* unread buffer held */
    BareMetal_Update(); assert(dma_starts==count+1 && dma_rx);
    finish(); BareMetal_Update();
    puts("PASS: DMA/unconsumed buffers protected; interrupts during DMA remain pending");
    uint32_t seen=g_imu_frame_count, errors=g_imu_error_count;
    data_ready=0U; frame(); data_ready=1U;
    assert(g_imu_frame_count==seen && g_imu_error_count==errors);
    frame(); assert(g_imu_frame_count==seen+1);
    puts("PASS: a stale INT_STATUS (no new sample) is ignored, not counted and not an error");
    uint32_t valid=g_imu_frame_count;
    raw[4]=INT16_MIN; frame(); assert(g_imu_frame_count==valid); stopped(); raw[4]=131;
    raw[0]=INT16_MIN; frame(); assert(g_imu_frame_count==valid); stopped(); raw[0]=426;
    fail_dma=1; request(); BareMetal_Update(); stopped(); BareMetal_Update();
    assert(!dma_rx && !MPU6050_IsBusy() && resets==1); fail_dma=0;
    request(); BareMetal_Update(); HAL_I2C_ErrorCallback(&hi2c2); BareMetal_Update(); stopped();
    request(); BareMetal_Update(); tick+=5; BareMetal_Update(); stopped(); assert(!dma_rx && resets==3);
    request(); BareMetal_Update(); finish(); tick+=5; BareMetal_Update(); stopped(); assert(resets==4);
    frame(); assert(last_motor.duty[0]>0); tick+=20; BareMetal_Update(); stopped();
    puts("PASS: invalid sample/start error/DMA error/timeout rejected, I2C2 reset; missing IMU stops motors");
    tick+=201; BareMetal_Update(); stopped(); radio(500); frame(); stopped();
    radio(0); radio(500); frame(); assert(last_motor.duty[0]>0);
    puts("PASS: radio loss stops motors; reconnect requires idle throttle");

    /* Isolate and exercise the real asynchronous OLED transport. */
    HAL_I2C_DeInit(&hi2c1); OLED_Init(); OLED_Clear(); OLED_DrawPoint(0,0); OLED_Update();
    OLED_Clear(); /* snapshot must retain this pixel */
    unsigned packets=0, pages=0;
    while (OLED_IsBusy())
    {
        OLED_Poll(); assert(oled_tx);
        if (oled_tx[0]==0x40)
        {
            assert(oled_size==129); if (pages==0) assert(oled_tx[1]==1); ++pages;
        }
        oled_tx=NULL; ++packets;
        HAL_I2C_MasterTxCpltCallback(&hi2c1);
        OLED_Poll();
        /* Poll may start the next packet, which is checked next iteration. */
    }
    assert(pages==8 && packets==17);
    OLED_Update(); OLED_Poll(); HAL_I2C_ErrorCallback(&hi2c1); OLED_Poll(); assert(!OLED_IsBusy() && !oled_tx);
    OLED_Update(); OLED_Poll(); tick+=50; OLED_Poll(); assert(!OLED_IsBusy() && !oled_tx);
    test_i2c_busy=1; OLED_Update(); OLED_Poll(); assert(!OLED_IsBusy());
    puts("PASS: OLED snapshot, 8 DMA pages, error/timeout cleanup and busy-bus skip");
    return 0;
}
