#ifndef ICM_TEST_MAIN_H
#define ICM_TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { int unused; } I2C_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
extern GPIO_TypeDef test_gpio;
#define GPIOB (&test_gpio)
#define GPIO_PIN_0 1U
#define GPIO_PIN_11 0x0800U
#define GPIO_PIN_12 0x1000U
#define I2C_FLAG_BUSY 1U
extern uint8_t test_i2c_busy;
#define __HAL_I2C_GET_FLAG(bus, flag) (test_i2c_busy)
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *, uint16_t, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *);
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *);
#endif
