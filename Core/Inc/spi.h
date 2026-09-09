/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   SPI peripheral configuration.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

void MX_SPI1_Init(void);
void MX_SPI2_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
