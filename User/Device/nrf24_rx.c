#include "nrf24_rx.h"

#define NRF24_CMD_R_REGISTER         0x00U
#define NRF24_CMD_W_REGISTER         0x20U
#define NRF24_CMD_R_RX_PAYLOAD       0x61U
#define NRF24_CMD_FLUSH_RX           0xE2U
#define NRF24_CMD_NOP                0xFFU
#define NRF24_SPI_TIMEOUT            10U

#define NRF24_REG_CONFIG             0x00U
#define NRF24_REG_EN_AA              0x01U
#define NRF24_REG_EN_RXADDR          0x02U
#define NRF24_REG_SETUP_AW           0x03U
#define NRF24_REG_RF_CH              0x05U
#define NRF24_REG_RF_SETUP           0x06U
#define NRF24_REG_STATUS             0x07U
#define NRF24_REG_RX_ADDR_P0         0x0AU
#define NRF24_REG_RX_PW_P0           0x11U
#define NRF24_REG_FIFO_STATUS        0x17U

#define NRF24_STATUS_RX_DR           (1U << 6)
/* RX_DR, TX_DS and MAX_RT all drive the IRQ line, and the line stays asserted
 * until every set flag is cleared. Clearing only RX_DR would leave it low. */
#define NRF24_STATUS_ALL_FLAGS       0x70U
#define NRF24_FIFO_RX_EMPTY          (1U << 0)

static const uint8_t nrf24_rx_address[5] =
{
  0xE7U, 0xE7U, 0xE7U, 0xE7U, 0x01U
};

static uint8_t NRF24_RX_IsHandleValid(const NRF24_RX_Handle_t *nrf);
static void NRF24_RX_Csn(const NRF24_RX_Handle_t *nrf, GPIO_PinState state);
static void NRF24_RX_Ce(const NRF24_RX_Handle_t *nrf, GPIO_PinState state);
static HAL_StatusTypeDef NRF24_RX_WriteRegister(NRF24_RX_Handle_t *nrf,
                                                 uint8_t reg,
                                                 uint8_t value);
static HAL_StatusTypeDef NRF24_RX_WriteBuffer(NRF24_RX_Handle_t *nrf,
                                               uint8_t reg,
                                               const uint8_t *data,
                                               uint8_t length);
static HAL_StatusTypeDef NRF24_RX_ReadRegister(NRF24_RX_Handle_t *nrf,
                                                uint8_t reg,
                                                uint8_t *value);
static HAL_StatusTypeDef NRF24_RX_Command(NRF24_RX_Handle_t *nrf, uint8_t command);

HAL_StatusTypeDef NRF24_RX_Init(NRF24_RX_Handle_t *nrf)
{
  uint8_t setup_aw = 0U;
  uint8_t rf_channel = 0U;

  if (NRF24_RX_IsHandleValid(nrf) == 0U)
  {
    return HAL_ERROR;
  }

  NRF24_RX_Ce(nrf, GPIO_PIN_RESET);
  NRF24_RX_Csn(nrf, GPIO_PIN_SET);
  HAL_Delay(5U);

  if ((NRF24_RX_WriteRegister(nrf, NRF24_REG_CONFIG, 0x0CU) != HAL_OK) ||
      (NRF24_RX_WriteRegister(nrf, NRF24_REG_EN_AA, 0x01U) != HAL_OK) ||
      (NRF24_RX_WriteRegister(nrf, NRF24_REG_EN_RXADDR, 0x01U) != HAL_OK) ||
      (NRF24_RX_WriteRegister(nrf, NRF24_REG_SETUP_AW, 0x03U) != HAL_OK) ||
      (NRF24_RX_WriteRegister(nrf, NRF24_REG_RF_CH, NRF24_RX_RF_CHANNEL) != HAL_OK) ||
      (NRF24_RX_WriteRegister(nrf, NRF24_REG_RF_SETUP, 0x06U) != HAL_OK) ||
      (NRF24_RX_WriteBuffer(nrf, NRF24_REG_RX_ADDR_P0, nrf24_rx_address, 5U) != HAL_OK) ||
      (NRF24_RX_WriteRegister(nrf, NRF24_REG_RX_PW_P0, NRF24_RX_PAYLOAD_SIZE) != HAL_OK) ||
      (NRF24_RX_WriteRegister(nrf, NRF24_REG_STATUS, 0x70U) != HAL_OK) ||
      (NRF24_RX_Command(nrf, NRF24_CMD_FLUSH_RX) != HAL_OK))
  {
    return HAL_ERROR;
  }

  if ((NRF24_RX_ReadRegister(nrf, NRF24_REG_SETUP_AW, &setup_aw) != HAL_OK) ||
      (NRF24_RX_ReadRegister(nrf, NRF24_REG_RF_CH, &rf_channel) != HAL_OK) ||
      (setup_aw != 0x03U) ||
      (rf_channel != NRF24_RX_RF_CHANNEL))
  {
    return HAL_ERROR;
  }

  if (NRF24_RX_WriteRegister(nrf, NRF24_REG_CONFIG, 0x0FU) != HAL_OK)
  {
    return HAL_ERROR;
  }

  HAL_Delay(2U);
  NRF24_RX_Ce(nrf, GPIO_PIN_SET);
  return HAL_OK;
}

HAL_StatusTypeDef NRF24_RX_IsDataReady(NRF24_RX_Handle_t *nrf, uint8_t *is_ready)
{
  uint8_t status = 0U;
  uint8_t fifo_status = 0U;

  if ((NRF24_RX_IsHandleValid(nrf) == 0U) || (is_ready == NULL))
  {
    return HAL_ERROR;
  }

  if ((NRF24_RX_ReadRegister(nrf, NRF24_REG_STATUS, &status) != HAL_OK) ||
      (NRF24_RX_ReadRegister(nrf, NRF24_REG_FIFO_STATUS, &fifo_status) != HAL_OK))
  {
    return HAL_ERROR;
  }

  *is_ready = (((status & NRF24_STATUS_RX_DR) != 0U) ||
               ((fifo_status & NRF24_FIFO_RX_EMPTY) == 0U)) ? 1U : 0U;
  return HAL_OK;
}

HAL_StatusTypeDef NRF24_RX_Read(NRF24_RX_Handle_t *nrf,
                                uint8_t data[NRF24_RX_PAYLOAD_SIZE])
{
  uint8_t command = NRF24_CMD_R_RX_PAYLOAD;
  HAL_StatusTypeDef status;

  if ((NRF24_RX_IsHandleValid(nrf) == 0U) || (data == NULL))
  {
    return HAL_ERROR;
  }

  NRF24_RX_Csn(nrf, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(nrf->hspi, &command, 1U, NRF24_SPI_TIMEOUT);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(nrf->hspi, data, NRF24_RX_PAYLOAD_SIZE, NRF24_SPI_TIMEOUT);
  }
  NRF24_RX_Csn(nrf, GPIO_PIN_SET);

  if (status != HAL_OK)
  {
    return status;
  }

  return NRF24_RX_WriteRegister(nrf, NRF24_REG_STATUS, NRF24_STATUS_ALL_FLAGS);
}

HAL_StatusTypeDef NRF24_RX_ClearInterrupts(NRF24_RX_Handle_t *nrf)
{
  if (NRF24_RX_IsHandleValid(nrf) == 0U)
  {
    return HAL_ERROR;
  }

  if (NRF24_RX_WriteRegister(nrf, NRF24_REG_STATUS, NRF24_STATUS_ALL_FLAGS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return NRF24_RX_Command(nrf, NRF24_CMD_FLUSH_RX);
}

static uint8_t NRF24_RX_IsHandleValid(const NRF24_RX_Handle_t *nrf)
{
  return ((nrf != NULL) && (nrf->hspi != NULL) &&
          (nrf->ce_gpio_port != NULL) && (nrf->csn_gpio_port != NULL)) ? 1U : 0U;
}

static void NRF24_RX_Csn(const NRF24_RX_Handle_t *nrf, GPIO_PinState state)
{
  HAL_GPIO_WritePin(nrf->csn_gpio_port, nrf->csn_pin, state);
}

static void NRF24_RX_Ce(const NRF24_RX_Handle_t *nrf, GPIO_PinState state)
{
  HAL_GPIO_WritePin(nrf->ce_gpio_port, nrf->ce_pin, state);
}

static HAL_StatusTypeDef NRF24_RX_WriteRegister(NRF24_RX_Handle_t *nrf,
                                                 uint8_t reg,
                                                 uint8_t value)
{
  uint8_t command = (uint8_t)(NRF24_CMD_W_REGISTER | reg);
  HAL_StatusTypeDef status;

  NRF24_RX_Csn(nrf, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(nrf->hspi, &command, 1U, NRF24_SPI_TIMEOUT);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Transmit(nrf->hspi, &value, 1U, NRF24_SPI_TIMEOUT);
  }
  NRF24_RX_Csn(nrf, GPIO_PIN_SET);
  return status;
}

static HAL_StatusTypeDef NRF24_RX_WriteBuffer(NRF24_RX_Handle_t *nrf,
                                               uint8_t reg,
                                               const uint8_t *data,
                                               uint8_t length)
{
  uint8_t command = (uint8_t)(NRF24_CMD_W_REGISTER | reg);
  HAL_StatusTypeDef status;

  NRF24_RX_Csn(nrf, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(nrf->hspi, &command, 1U, NRF24_SPI_TIMEOUT);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Transmit(nrf->hspi, (uint8_t *)data, length, NRF24_SPI_TIMEOUT);
  }
  NRF24_RX_Csn(nrf, GPIO_PIN_SET);
  return status;
}

static HAL_StatusTypeDef NRF24_RX_ReadRegister(NRF24_RX_Handle_t *nrf,
                                                uint8_t reg,
                                                uint8_t *value)
{
  uint8_t command = (uint8_t)(NRF24_CMD_R_REGISTER | reg);
  uint8_t dummy = NRF24_CMD_NOP;
  HAL_StatusTypeDef status;

  NRF24_RX_Csn(nrf, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(nrf->hspi, &command, 1U, NRF24_SPI_TIMEOUT);
  if (status == HAL_OK)
  {
    status = HAL_SPI_TransmitReceive(nrf->hspi, &dummy, value, 1U, NRF24_SPI_TIMEOUT);
  }
  NRF24_RX_Csn(nrf, GPIO_PIN_SET);
  return status;
}

static HAL_StatusTypeDef NRF24_RX_Command(NRF24_RX_Handle_t *nrf, uint8_t command)
{
  HAL_StatusTypeDef status;

  NRF24_RX_Csn(nrf, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(nrf->hspi, &command, 1U, NRF24_SPI_TIMEOUT);
  NRF24_RX_Csn(nrf, GPIO_PIN_SET);
  return status;
}
