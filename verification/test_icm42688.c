/* Actual driver, baremetal scheduler, radio parser, estimator/controller and
 * OLED state machine. Hardware is mocked; this is not a timing/flight test. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "baremetal.h"
#include "icm42688.h"
#include "imu_task.h"
#include "motor.h"
#include "OLED.h"

SPI_HandleTypeDef hspi2;
I2C_HandleTypeDef hi2c1;
GPIO_TypeDef test_gpio;
uint8_t test_i2c_busy;
static uint32_t tick, primask, dma_starts, applies, aborts;
static uint8_t regs[128], *dma_rx, *oled_tx, packet[20], packet_ready;
static uint16_t oled_size;
static int16_t raw[7] = {0, 0, 0, 4096, 131, -262, 393};
static GPIO_PinState cs = GPIO_PIN_SET;
static uint8_t fail_dma, fail_reg, corrupt_reg;
static Motor_Output_t last_motor;
void HAL_GPIO_EXTI_Callback(uint16_t);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *);

uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1U; }
void __set_PRIMASK(uint32_t p) { primask = p; }
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { assert(!primask); tick += ms; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t n)
{ (void)p; (void)n; return GPIO_PIN_SET; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t n, GPIO_PinState s)
{ assert(p == GPIOB && n == ICM42688_CS_PIN); cs = s; }

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *s, uint8_t *tx,
    uint8_t *rx, uint16_t n, uint32_t timeout)
{
    (void)timeout;
    assert(!primask && s == &hspi2 && cs == GPIO_PIN_RESET && !dma_rx && n == 2U);
    uint8_t reg = tx[0] & 0x7FU;
    if (fail_reg == reg) return HAL_ERROR;
    if (tx[0] & 0x80U)
    {
        rx[1] = regs[reg] ^ (corrupt_reg == reg ? 1U : 0U);
        if (reg == 0x2DU) regs[reg] = 0U;
    }
    else
    {
        regs[reg] = tx[1];
        if (reg == 0x11U && tx[1] == 1U) regs[0x2D] = 0x10U;
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *s, uint8_t *tx,
    uint8_t *rx, uint16_t n)
{
    assert(!primask && s == &hspi2 && cs == GPIO_PIN_RESET && !dma_rx);
    assert(n == 15U && tx[0] == 0x9DU);
    for (unsigned i=1; i<n; ++i) assert(tx[i] == 0U);
    dma_rx = rx;
    ++dma_starts;
    return fail_dma ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *s)
{ assert(s == &hspi2); dma_rx = NULL; ++aborts; return HAL_OK; }

HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *b, uint16_t addr,
    uint8_t *data, uint16_t n)
{
    assert(b == &hi2c1 && addr == 0x78U && !oled_tx && n > 1U);
    oled_tx = data; oled_size = n; return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *b)
{ assert(b == &hi2c1); oled_tx = NULL; return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *b)
{ assert(b == &hi2c1); return HAL_OK; }
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
    regs[0x2D] |= 0x08U;
    uint32_t before = dma_starts;
    HAL_GPIO_EXTI_Callback(ICM42688_INT_PIN);
    assert(dma_starts == before); /* EXTI never starts SPI. */
}
static void finish(void)
{
    assert(dma_rx);
    dma_rx[0] = 0xFFU; /* command phase must be discarded */
    for (unsigned i=0; i<7; ++i)
    {
        dma_rx[1+2*i] = (uint8_t)((uint16_t)raw[i] >> 8);
        dma_rx[2+2*i] = (uint8_t)raw[i];
    }
    dma_rx = NULL;
    HAL_SPI_TxRxCpltCallback(&hspi2);
    assert(cs == GPIO_PIN_SET);
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
    assert(ICM42688_Init() == HAL_ERROR); /* wrong WHO_AM_I */
    regs[0x75] = 0x47;
    fail_reg = 0x4F; assert(ICM42688_Init() == HAL_ERROR); fail_reg = 0;
    corrupt_reg = 0x50; assert(ICM42688_Init() == HAL_ERROR); corrupt_reg = 0;
    assert(BareMetal_Init(&cfg) == HAL_OK);
    assert(regs[0x4F]==0x46 && regs[0x50]==0x26 && regs[0x4E]==0x0F);
    assert(regs[0x16]==0 && regs[0x14]==3 && regs[0x64]==0 && regs[0x65]==8);
    puts("PASS: identity/readback failures rejected; 1 kHz/LN/FIFO bypass/DRDY configured");
    for (unsigned i=1; i<=1000; ++i)
    {
        if (i%100==1) radio(0);
        frame();
        assert(g_gyro_calibrated == (i==1000) && applies==0);
    }
    for (unsigned i=0;i<3;++i) assert(fabsf(g_imu_data.gyro_dps[i]) < 0.0001f);
    assert(g_imu_data.acc_g[2] == 1.0f && g_imu_data.temperature_deg_c == 25.0f);
    for (unsigned i=1;i<=500;++i)
    {
        if (i%100==1) radio(0);
        frame(); assert(g_attitude_zero_calibrated==(i==500) && applies==0);
    }
    radio(500); frame(); assert(applies==1 && last_motor.duty[0]>0);
    BareMetal_Update(); assert(applies==1);
    puts("PASS: 1000 DMA frames remove gyro bias; 500 attitude samples; no premature/stale control");
    request(); BareMetal_Update(); assert(ICM42688_Consume(&sample)==HAL_BUSY);
    request(); request(); assert(g_imu_coalesced_count>0);
    uint32_t count=dma_starts; BareMetal_Update(); assert(dma_starts==count);
    finish(); assert(ICM42688_StartDmaRead()==HAL_BUSY); /* unread buffer held */
    BareMetal_Update(); assert(dma_starts==count+1 && dma_rx);
    finish(); BareMetal_Update();
    puts("PASS: DMA/unconsumed buffers protected; interrupts during DMA remain pending");
    uint32_t valid=g_imu_frame_count;
    raw[4]=INT16_MIN; frame(); assert(g_imu_frame_count==valid); stopped(); raw[4]=131;
    fail_dma=1; request(); BareMetal_Update(); stopped(); BareMetal_Update();
    assert(!dma_rx && !ICM42688_IsBusy()); fail_dma=0;
    request(); BareMetal_Update(); HAL_SPI_ErrorCallback(&hspi2); BareMetal_Update(); stopped();
    request(); BareMetal_Update(); tick+=5; BareMetal_Update(); stopped(); assert(!dma_rx && aborts>=3);
    request(); BareMetal_Update(); finish(); tick+=5; BareMetal_Update(); stopped();
    frame(); assert(last_motor.duty[0]>0); tick+=20; BareMetal_Update(); stopped();
    puts("PASS: invalid sample/start error/DMA error/timeout rejected; missing IMU stops motors");
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
