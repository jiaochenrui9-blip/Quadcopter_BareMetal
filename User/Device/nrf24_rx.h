#ifndef __NRF24_RX_H__
#define __NRF24_RX_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f1xx_hal_spi.h"

#define NRF24_RX_RF_CHANNEL        40U
#define NRF24_RX_PAYLOAD_SIZE      20U

typedef struct
{
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *ce_gpio_port;
  uint16_t ce_pin;
  GPIO_TypeDef *csn_gpio_port;
  uint16_t csn_pin;
} NRF24_RX_Handle_t;

HAL_StatusTypeDef NRF24_RX_Init(NRF24_RX_Handle_t *nrf);
HAL_StatusTypeDef NRF24_RX_IsDataReady(NRF24_RX_Handle_t *nrf, uint8_t *is_ready);
HAL_StatusTypeDef NRF24_RX_Read(NRF24_RX_Handle_t *nrf,
                                uint8_t data[NRF24_RX_PAYLOAD_SIZE]);

/* Recovery path: clears every latched interrupt flag and empties the RX FIFO,
 * which releases the IRQ line so edge-triggered handoff can resume. */
HAL_StatusTypeDef NRF24_RX_ClearInterrupts(NRF24_RX_Handle_t *nrf);

#ifdef __cplusplus
}
#endif

#endif /* __NRF24_RX_H__ */
